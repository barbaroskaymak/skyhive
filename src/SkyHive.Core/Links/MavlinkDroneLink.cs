using System.Collections.Concurrent;
using System.Diagnostics;
using SkyHive.Core.Geo;
using SkyHive.Core.Mavlink;
using SkyHive.Core.Model;

namespace SkyHive.Core.Links;

/// <summary>Controls a real ArduCopter (or ArduPilot SITL) over MAVLink 2.</summary>
public sealed class MavlinkDroneLink : IDroneLink
{
    readonly IMavTransport _transport;
    readonly MavlinkParser _parser = new();
    readonly MavlinkWriter _writer = new(255, 190);
    readonly ConcurrentDictionary<ushort, TaskCompletionSource<MavResult>> _pendingAcks = new();
    readonly SemaphoreSlim _sendLock = new(1, 1);
    readonly Stopwatch _clock = Stopwatch.StartNew();
    CancellationTokenSource? _cts;
    Task? _rx, _hb;
    byte _targetSys = 1, _targetComp = 1;
    bool _streamsRequested;
    DateTime _lastStreamRequest;

    public MavlinkDroneLink(int id, IMavTransport transport)
    {
        Id = id;
        _transport = transport;
    }

    public int Id { get; }
    public string Name => $"D{Id}";
    public string Endpoint => _transport.Description;
    public DroneState State { get; } = new();
    public event Action<IDroneLink, LogLevel, string>? Log;
    public long CrcErrors => _parser.CrcErrors;

    void Say(LogLevel level, string text) => Log?.Invoke(this, level, text);

    public Task StartAsync(CancellationToken ct)
    {
        _cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        _rx = Task.Run(() => ReceiveLoopAsync(_cts.Token));
        _hb = Task.Run(() => HeartbeatLoopAsync(_cts.Token));
        return Task.CompletedTask;
    }

    async Task ReceiveLoopAsync(CancellationToken ct)
    {
        var buffer = new byte[4096];
        while (!ct.IsCancellationRequested)
        {
            try
            {
                if (!_transport.IsOpen)
                {
                    await _transport.OpenAsync(ct);
                    Say(LogLevel.Info, $"Bağlantı açıldı: {_transport.Description}");
                    _streamsRequested = false;
                }
                int n = await _transport.ReceiveAsync(buffer, ct);
                if (n <= 0)
                {
                    State.Connected = false;
                    Say(LogLevel.Warning, "Bağlantı koptu, yeniden deneniyor…");
                    await _transport.DisposeAsync();
                    await Task.Delay(1000, ct);
                    continue;
                }
                foreach (var frame in _parser.Push(buffer.AsSpan(0, n))) Handle(frame);
            }
            catch (OperationCanceledException) { break; }
            catch (Exception ex)
            {
                State.Connected = false;
                Say(LogLevel.Warning, $"{_transport.Description}: {ex.Message} — 2 sn sonra tekrar");
                try { await _transport.DisposeAsync(); await Task.Delay(2000, ct); } catch (OperationCanceledException) { break; }
            }
        }
    }

    async Task HeartbeatLoopAsync(CancellationToken ct)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(1));
        while (await timer.WaitForNextTickAsync(ct).ConfigureAwait(false))
        {
            try
            {
                if (!_transport.IsOpen) continue;
                await SendAsync(MavMsg.Heartbeat, MavCodec.EncodeHeartbeat(Heartbeat.TypeGcs, 8, 0, 0, 4));
                if (State.Connected && (DateTime.UtcNow - State.LastHeartbeat).TotalSeconds > 5)
                {
                    State.Connected = false;
                    Say(LogLevel.Warning, "Heartbeat zaman aşımı");
                }
                if (State.Connected && (!_streamsRequested || (DateTime.UtcNow - _lastStreamRequest).TotalSeconds > 30 && !State.HasPosition))
                    await RequestStreamsAsync();
            }
            catch (OperationCanceledException) { break; }
            catch { /* transport hiccup: receive loop reconnects */ }
        }
    }

    async Task RequestStreamsAsync()
    {
        _streamsRequested = true;
        _lastStreamRequest = DateTime.UtcNow;
        // MAV_DATA_STREAM_ALL at 4 Hz, position at 10 Hz
        await SendAsync(MavMsg.RequestDataStream, MavCodec.EncodeRequestDataStream(_targetSys, _targetComp, 0, 4, true));
        await SendAsync(MavMsg.CommandLong, MavCodec.EncodeCommandLong(_targetSys, _targetComp, MavCmd.SetMessageInterval, 0, MavMsg.GlobalPositionInt, 100_000));
        await SendAsync(MavMsg.CommandLong, MavCodec.EncodeCommandLong(_targetSys, _targetComp, MavCmd.GetHomePosition, 0));
    }

    void Handle(MavlinkFrame f)
    {
        switch (f.MsgId)
        {
            case MavMsg.Heartbeat:
                var hb = MavCodec.DecodeHeartbeat(f);
                if (hb.Type == Heartbeat.TypeGcs || hb.Autopilot == 8) return; // other GCS / non-autopilot component
                _targetSys = f.SysId;
                _targetComp = f.CompId;
                if (!State.Connected) Say(LogLevel.Success, $"ArduPilot bağlı (sysid {f.SysId})");
                State.Connected = true;
                State.LastHeartbeat = DateTime.UtcNow;
                if (State.Armed != hb.Armed) Say(hb.Armed ? LogLevel.Success : LogLevel.Info, hb.Armed ? "ARMED" : "DISARMED");
                State.Armed = hb.Armed;
                var newMode = Enum.IsDefined(typeof(FlightMode), (int)hb.CustomMode) ? (FlightMode)hb.CustomMode : FlightMode.Unknown;
                State.RawMode = hb.CustomMode;
                if (State.Mode != newMode && State.Mode != FlightMode.Unknown) Say(LogLevel.Info, $"Mod: {newMode.ToString().ToUpperInvariant()}");
                State.Mode = newMode;
                break;
            case MavMsg.GlobalPositionInt:
                if (f.SysId != _targetSys) return;
                var gp = MavCodec.DecodeGlobalPosition(f);
                if (gp.LatE7 == 0 && gp.LonE7 == 0) return;
                State.Position = new GeoPoint(gp.LatE7 / 1e7, gp.LonE7 / 1e7, gp.RelAltMm / 1000.0);
                State.HasPosition = true;
                State.Home ??= State.Position with { Alt = 0 };
                if (gp.HdgCdeg != ushort.MaxValue) State.Heading = gp.HdgCdeg / 100.0;
                State.GroundSpeed = Math.Sqrt(gp.Vx * gp.Vx + gp.Vy * gp.Vy) / 100.0;
                State.ClimbRate = -gp.Vz / 100.0;
                break;
            case MavMsg.VfrHud:
                var hud = MavCodec.DecodeVfrHud(f);
                State.GroundSpeed = hud.GroundSpeed;
                State.ClimbRate = hud.Climb;
                State.Heading = hud.Heading;
                break;
            case MavMsg.Attitude:
                var att = MavCodec.DecodeAttitude(f);
                State.Roll = att.Roll;
                State.Pitch = att.Pitch;
                break;
            case MavMsg.SysStatus:
                var ss = MavCodec.DecodeSysStatus(f);
                State.BatteryVoltage = ss.VoltageMv / 1000.0;
                if (ss.BatteryRemaining >= 0) State.BatteryPercent = ss.BatteryRemaining;
                break;
            case MavMsg.GpsRawInt:
                var gps = MavCodec.DecodeGpsRaw(f);
                State.GpsFix = gps.FixType;
                State.Satellites = gps.Satellites;
                break;
            case MavMsg.HomePosition:
                var home = MavCodec.DecodeHomePosition(f);
                State.Home = new GeoPoint(home.LatE7 / 1e7, home.LonE7 / 1e7, 0);
                break;
            case MavMsg.CommandAck:
                var ack = MavCodec.DecodeCommandAck(f);
                if (ack.Result != MavResult.InProgress && _pendingAcks.TryRemove(ack.Command, out var tcs))
                    tcs.TrySetResult(ack.Result);
                break;
            case MavMsg.StatusText:
                var st = MavCodec.DecodeStatusText(f);
                State.LastStatusText = st.Text;
                Say(st.Severity <= 3 ? LogLevel.Error : st.Severity == 4 ? LogLevel.Warning : LogLevel.Info, st.Text);
                break;
        }
    }

    async Task SendAsync(uint msgId, byte[] payload)
    {
        await _sendLock.WaitAsync();
        try { await _transport.SendAsync(_writer.Build(msgId, payload), CancellationToken.None); }
        finally { _sendLock.Release(); }
    }

    /// <summary>Sends COMMAND_LONG and waits for COMMAND_ACK, retrying with an increasing confirmation field.</summary>
    public async Task<MavResult> CommandAsync(ushort command, float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0)
    {
        for (byte attempt = 0; attempt < 3; attempt++)
        {
            var tcs = new TaskCompletionSource<MavResult>(TaskCreationOptions.RunContinuationsAsynchronously);
            _pendingAcks[command] = tcs;
            await SendAsync(MavMsg.CommandLong, MavCodec.EncodeCommandLong(_targetSys, _targetComp, command, attempt, p1, p2, p3, p4, p5, p6, p7));
            var done = await Task.WhenAny(tcs.Task, Task.Delay(1500));
            if (done == tcs.Task) return tcs.Task.Result;
        }
        _pendingAcks.TryRemove(command, out _);
        return MavResult.Failed;
    }

    public async Task<bool> SetModeAsync(FlightMode mode)
    {
        var r = await CommandAsync(MavCmd.DoSetMode, 1 /* CUSTOM_MODE_ENABLED */, (float)(int)mode);
        if (r != MavResult.Accepted) Say(LogLevel.Warning, $"Mod {mode} reddedildi ({r})");
        return r == MavResult.Accepted;
    }

    public async Task<bool> ArmAsync(bool arm)
    {
        var r = await CommandAsync(MavCmd.ComponentArmDisarm, arm ? 1 : 0);
        if (r != MavResult.Accepted) Say(LogLevel.Warning, $"{(arm ? "Arm" : "Disarm")} reddedildi ({r})");
        return r == MavResult.Accepted;
    }

    public async Task<bool> TakeoffAsync(double altitude)
    {
        if (State.Mode != FlightMode.Guided && !await SetModeAsync(FlightMode.Guided)) return false;
        if (State.Airborne)
        {
            await GotoAsync(State.Position with { Alt = altitude });
            return true;
        }
        // SITL needs ~20-40 s after boot for EKF/GPS before pre-arm checks pass: keep trying.
        var deadline = DateTime.UtcNow.AddSeconds(60);
        while (!State.Armed)
        {
            if (await ArmAsync(true)) break;
            if (DateTime.UtcNow > deadline) return false;
            await Task.Delay(3000);
        }
        var r = await CommandAsync(MavCmd.NavTakeoff, 0, 0, 0, float.NaN, 0, 0, (float)altitude);
        if (r != MavResult.Accepted) { Say(LogLevel.Warning, $"Takeoff reddedildi ({r})"); return false; }
        Say(LogLevel.Success, $"Takeoff {altitude:F0} m");
        return true;
    }

    public async Task GotoAsync(GeoPoint target)
    {
        if (State.Mode is FlightMode.Rtl or FlightMode.Land) return;
        if (State.Mode != FlightMode.Guided) await SetModeAsync(FlightMode.Guided);
        await SendAsync(MavMsg.SetPositionTargetGlobalInt, MavCodec.EncodeSetPositionTargetGlobalInt(
            _targetSys, _targetComp, (uint)_clock.ElapsedMilliseconds,
            (int)Math.Round(target.Lat * 1e7), (int)Math.Round(target.Lon * 1e7), (float)target.Alt));
    }

    public Task<bool> LandAsync() => SetModeAsync(FlightMode.Land);
    public Task<bool> ReturnToLaunchAsync() => SetModeAsync(FlightMode.Rtl);

    public async Task SetSpeedAsync(double metresPerSecond) =>
        await CommandAsync(MavCmd.DoChangeSpeed, 1, (float)metresPerSecond, -1);

    public async ValueTask DisposeAsync()
    {
        _cts?.Cancel();
        foreach (var t in new[] { _rx, _hb })
            if (t is not null) { try { await t; } catch { } }
        await _transport.DisposeAsync();
        State.Connected = false;
    }
}
