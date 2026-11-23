using SkyHive.Core.Geo;
using SkyHive.Core.Model;

namespace SkyHive.Core.Links;

/// <summary>
/// Built-in multicopter simulator that mimics ArduCopter behaviour (modes, pre-arm, takeoff, guided targets,
/// RTL/LAND, battery failsafe). Lets the app run without ArduPilot SITL installed.
/// </summary>
public sealed class SimDroneLink : IDroneLink
{
    readonly object _gate = new();
    readonly GeoPoint _home;
    readonly Random _rng;
    CancellationTokenSource? _cts;
    Task? _loop;

    // local state (metres relative to home)
    double _n, _e, _u, _vn, _ve, _vu, _heading;
    bool _armed;
    FlightMode _mode = FlightMode.Stabilize;
    double? _takeoffAlt;
    (double N, double E, double U)? _target;
    (double N, double E, double U) _hold;
    int _rtlPhase;
    double _speed = 6;
    double _battery = 100;
    double _bootTime;
    double _landedArmedTime;
    double _windN, _windE;

    public SimDroneLink(int id, GeoPoint home, int seed = 0, double headingDeg = 0)
    {
        Id = id;
        _home = home;
        _heading = headingDeg;
        _rng = new Random(seed == 0 ? id * 7919 : seed);
        State.Home = home;
        State.Position = home with { Alt = 0 };
        State.HasPosition = true;
    }

    public int Id { get; }
    public string Name => $"D{Id}";
    public string Endpoint => "sim";
    public DroneState State { get; } = new();
    public double TimeScale { get; set; } = 1.0;
    public event Action<IDroneLink, LogLevel, string>? Log;

    void Say(string text, LogLevel level = LogLevel.Info)
    {
        State.LastStatusText = text;
        Log?.Invoke(this, level, text);
    }

    public Task StartAsync(CancellationToken ct)
    {
        _cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        State.Connected = true;
        State.LastHeartbeat = DateTime.UtcNow;
        _loop = Task.Run(() => RunAsync(_cts.Token));
        Say("ArduCopter V4.5 (SkyHive sim) hazır");
        return Task.CompletedTask;
    }

    async Task RunAsync(CancellationToken ct)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(50));
        while (await timer.WaitForNextTickAsync(ct).ConfigureAwait(false))
            Step(0.05 * TimeScale);
    }

    /// <summary>Advances the physics by <paramref name="dt"/> seconds (public for deterministic tests).</summary>
    public void Step(double dt)
    {
        List<(string, LogLevel)>? messages = null;
        void Msg(string t, LogLevel l = LogLevel.Info) => (messages ??= new()).Add((t, l));

        lock (_gate)
        {
            _bootTime += dt;
            var airborne = _u > 0.05;
            (double N, double E, double U) desired = (_n, _e, _u);
            double maxDown = 1.5;

            switch (_mode)
            {
                case FlightMode.Guided when _armed:
                    if (_takeoffAlt is { } ta)
                    {
                        desired = (_n, _e, ta);
                        if (Math.Abs(_u - ta) < 0.25)
                        {
                            _takeoffAlt = null;
                            _target ??= (_n, _e, ta);
                        }
                    }
                    else if (_target is { } t) desired = t;
                    else desired = airborne ? _hold : (_n, _e, 0);
                    break;
                case FlightMode.Loiter or FlightMode.Brake or FlightMode.PosHold or FlightMode.AltHold or FlightMode.Auto when _armed:
                    desired = _hold;
                    break;
                case FlightMode.Land when _armed:
                    desired = (_n, _e, -1);
                    maxDown = _u > 10 ? 2.5 : 0.8;
                    break;
                case FlightMode.Rtl when _armed:
                    var rtlAlt = Math.Max(15, _hold.U);
                    var distHome = Math.Sqrt(_n * _n + _e * _e);
                    if (_rtlPhase == 0)
                    {
                        desired = (_n, _e, rtlAlt);
                        if (_u >= rtlAlt - 0.5) _rtlPhase = 1;
                    }
                    else if (_rtlPhase == 1)
                    {
                        desired = (0, 0, rtlAlt);
                        if (distHome < 1.0) { _rtlPhase = 2; Msg("RTL: eve varıldı, iniş"); }
                    }
                    else
                    {
                        desired = (0, 0, -1);
                        maxDown = _u > 10 ? 2.5 : 0.8;
                    }
                    break;
            }

            if (_armed)
            {
                Fly(desired, dt, maxDown, airborne);
                // battery drain: hover + forward-flight + climb
                var hs = Math.Sqrt(_vn * _vn + _ve * _ve);
                _battery = Math.Max(0, _battery - dt * (0.045 + 0.006 * hs + 0.02 * Math.Max(0, _vu)));
                if (_battery < 15 && _mode is not (FlightMode.Rtl or FlightMode.Land) && airborne)
                {
                    _mode = FlightMode.Rtl;
                    _rtlPhase = 0;
                    _hold = (_n, _e, _u);
                    Msg("Battery failsafe: RTL", LogLevel.Warning);
                }

                if (_u <= 0.0 && !airborne)
                {
                    _landedArmedTime += dt;
                    if (_mode == FlightMode.Land || (_mode == FlightMode.Rtl && _rtlPhase == 2))
                    {
                        _armed = false;
                        _mode = _mode == FlightMode.Rtl ? FlightMode.Land : _mode;
                        Msg("Land complete — Disarming motors", LogLevel.Success);
                    }
                    else if (_landedArmedTime > 15 && _takeoffAlt is null)
                    {
                        _armed = false;
                        Msg("Auto-disarm (yerde boşta)");
                    }
                }
                else _landedArmedTime = 0;
            }
            else
            {
                _vn = _ve = _vu = 0;
                if (_u > 0) _u = Math.Max(0, _u - 9.8 * dt); // unpowered: fall
            }

            Publish();
        }

        if (messages is not null)
            foreach (var (t, l) in messages) Say(t, l);
    }

    void Fly((double N, double E, double U) d, double dt, double maxDown, bool airborne)
    {
        const double accel = 3.5, vaccel = 3.0, maxUp = 2.5;
        var dn = d.N - _n; var de = d.E - _e;
        var dist = Math.Sqrt(dn * dn + de * de);
        double wantN = 0, wantE = 0;
        if (airborne && dist > 0.05)
        {
            var sp = Math.Min(_speed, Math.Sqrt(2 * accel * 0.8 * dist)); // decelerate on approach
            wantN = dn / dist * sp; wantE = de / dist * sp;
        }
        var dvn = wantN - _vn; var dve = wantE - _ve;
        var dv = Math.Sqrt(dvn * dvn + dve * dve);
        var maxDv = accel * dt;
        if (dv > maxDv) { dvn *= maxDv / dv; dve *= maxDv / dv; }
        _vn += dvn; _ve += dve;

        var du = d.U - _u;
        var wantU = Math.Clamp(du * 1.2, -maxDown, maxUp);
        _vu += Math.Clamp(wantU - _vu, -vaccel * dt, vaccel * dt);

        // gusty wind when airborne
        if (airborne)
        {
            _windN += (_rng.NextDouble() - 0.5) * 0.3 * dt - _windN * 0.2 * dt;
            _windE += (_rng.NextDouble() - 0.5) * 0.3 * dt - _windE * 0.2 * dt;
        }
        _n += (_vn + (airborne ? _windN : 0)) * dt;
        _e += (_ve + (airborne ? _windE : 0)) * dt;
        _u += _vu * dt;
        if (_u < 0) { _u = 0; _vu = 0; _vn *= 0.5; _ve *= 0.5; }

        var hs = Math.Sqrt(_vn * _vn + _ve * _ve);
        if (hs > 0.6)
        {
            var course = GeoMath.NormalizeDeg(Math.Atan2(_ve, _vn) * 180 / Math.PI);
            var diff = GeoMath.AngleDiff(_heading, course);
            _heading = GeoMath.NormalizeDeg(_heading + Math.Clamp(diff, -150 * dt, 150 * dt));
        }
        State.Pitch = -Math.Clamp(dvn / Math.Max(dt, 1e-3) / 9.8, -0.5, 0.5);
        State.Roll = Math.Clamp(dve / Math.Max(dt, 1e-3) / 9.8, -0.5, 0.5);
    }

    void Publish()
    {
        State.Position = GeoMath.Offset(_home, _n, _e, _u);
        State.Heading = _heading;
        State.GroundSpeed = Math.Sqrt(_vn * _vn + _ve * _ve);
        State.ClimbRate = _vu;
        State.Armed = _armed;
        State.Mode = _mode;
        State.RawMode = (uint)_mode;
        State.BatteryPercent = _battery;
        State.BatteryVoltage = 10.5 + 2.1 * _battery / 100.0;
        State.GpsFix = _bootTime > 1.5 ? 3 : 0;
        State.Satellites = _bootTime > 1.5 ? 12 + Id % 5 : 0;
        State.LastHeartbeat = DateTime.UtcNow;
    }

    public Task<bool> SetModeAsync(FlightMode mode)
    {
        lock (_gate)
        {
            _mode = mode;
            _hold = (_n, _e, _u);
            _rtlPhase = 0;
            if (mode != FlightMode.Guided) { _target = null; _takeoffAlt = null; }
            else _target = null;
            Publish();
        }
        Say($"Mode {mode.ToString().ToUpperInvariant()}");
        return Task.FromResult(true);
    }

    public Task<bool> ArmAsync(bool arm)
    {
        string? error = null;
        lock (_gate)
        {
            if (arm)
            {
                if (_bootTime < 1.5) error = "PreArm: GPS bekleniyor (3D fix yok)";
                else if (_battery < 20) error = "PreArm: Batarya düşük";
                else if (_mode is FlightMode.Rtl or FlightMode.Land or FlightMode.Auto) error = $"Arm: {_mode} modunda arm edilemez";
                else { _armed = true; _landedArmedTime = 0; Publish(); }
            }
            else
            {
                if (_u > 0.5) error = "Disarm reddedildi: havada";
                else { _armed = false; Publish(); }
            }
        }
        if (error is not null) { Say(error, LogLevel.Warning); return Task.FromResult(false); }
        Say(arm ? "Arming motors" : "Disarming motors", LogLevel.Success);
        return Task.FromResult(true);
    }

    public async Task<bool> TakeoffAsync(double altitude)
    {
        if (State.Mode != FlightMode.Guided) await SetModeAsync(FlightMode.Guided);
        for (int i = 0; i < 20 && !State.Armed; i++)
        {
            if (await ArmAsync(true)) break;
            await Task.Delay(500);
        }
        lock (_gate)
        {
            if (!_armed) return false;
            if (_u > 1) { _target = (_n, _e, altitude); }
            else _takeoffAlt = altitude;
        }
        Say($"Takeoff {altitude:F0} m", LogLevel.Success);
        return true;
    }

    public Task GotoAsync(GeoPoint target)
    {
        var local = GeoMath.ToLocal(_home, target);
        lock (_gate)
        {
            if (_mode is FlightMode.Rtl or FlightMode.Land) return Task.CompletedTask; // like ArduCopter: targets only apply in GUIDED
            if (_mode != FlightMode.Guided)
            {
                _mode = FlightMode.Guided;
                _rtlPhase = 0;
            }
            if (_takeoffAlt is not null && _u < _takeoffAlt - 1)
                _takeoffAlt = Math.Max(_takeoffAlt.Value, target.Alt);
            _target = (local.N, local.E, Math.Max(1, target.Alt));
            _hold = _target.Value;
        }
        return Task.CompletedTask;
    }

    public async Task<bool> LandAsync() => await SetModeAsync(FlightMode.Land);
    public async Task<bool> ReturnToLaunchAsync() => await SetModeAsync(FlightMode.Rtl);

    public Task SetSpeedAsync(double metresPerSecond)
    {
        lock (_gate) _speed = Math.Clamp(metresPerSecond, 0.5, 20);
        return Task.CompletedTask;
    }

    public async ValueTask DisposeAsync()
    {
        _cts?.Cancel();
        if (_loop is not null) { try { await _loop; } catch (OperationCanceledException) { } }
        State.Connected = false;
    }
}
