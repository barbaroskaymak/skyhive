using SkyHive.Core.Commands;
using SkyHive.Core.Geo;
using SkyHive.Core.Links;
using SkyHive.Core.Mavlink;
using SkyHive.Core.Model;
using SkyHive.Core.Swarm;
using SkyHive.Core.Vision;

namespace SkyHive.Core;

public enum LinkMode { Simulation, SitlTcp, SitlUdp }

public sealed record SessionOptions
{
    public LinkMode Mode { get; init; } = LinkMode.Simulation;
    public int DroneCount { get; init; } = 5;
    public string Host { get; init; } = "127.0.0.1";
    /// <summary>SITL instance N listens on BasePort + PortStride × N (ArduPilot default 5760 + 10N).</summary>
    public int BasePort { get; init; } = 5760;
    public int PortStride { get; init; } = 10;
    /// <summary>Same default home as sitl/start-swarm.sh so the synthetic world lines up.</summary>
    public GeoPoint Origin { get; init; } = new(40.9925, 29.1240, 0);
    public int WorldSeed { get; init; } = 1337;
}

/// <summary>Top-level façade: links + controller + vision + tracker + command interpreter.</summary>
public sealed class SwarmSession : IAsyncDisposable
{
    readonly CancellationTokenSource _cts = new();
    readonly List<Task> _loops = new();
    readonly List<LogEntry> _log = new();
    readonly object _logGate = new();

    SwarmSession(SessionOptions options, IReadOnlyList<IDroneLink> links)
    {
        Options = options;
        Origin = options.Origin;
        World = new SyntheticWorld(options.WorldSeed);
        Tracker = new TargetTracker();
        Agents = links.Select((l, i) => new DroneAgent(l, i, () => Origin)).ToList();
        Controller = new SwarmController(Agents, Tracker);
        Vision = new VisionService(World, Tracker, () => Agents);
        Commands = new CommandInterpreter(this);

        foreach (var l in links) l.Log += (link, level, msg) => Write(level, link.Name, msg);
        Controller.Log += (level, src, msg) => Write(level, src, msg);
        Tracker.TargetConfirmed += t => Write(LogLevel.Success, t.LastSeenBy, $"Hedef tespit edildi {t.Label} @ {t.Position} (güven %{t.Confidence * 100:F0})");
    }

    public SessionOptions Options { get; }
    public GeoPoint Origin { get; private set; }
    public SyntheticWorld World { get; }
    public TargetTracker Tracker { get; }
    public IReadOnlyList<DroneAgent> Agents { get; }
    public SwarmController Controller { get; }
    public VisionService Vision { get; }
    public CommandInterpreter Commands { get; }
    public event Action<LogEntry>? Logged;

    public IReadOnlyList<LogEntry> LogSnapshot()
    {
        lock (_logGate) return _log.ToList();
    }

    public void Write(LogLevel level, string source, string message)
    {
        var e = new LogEntry(DateTime.Now, level, source, message);
        lock (_logGate)
        {
            _log.Add(e);
            if (_log.Count > 500) _log.RemoveRange(0, 100);
        }
        Logged?.Invoke(e);
    }

    public static async Task<SwarmSession> StartAsync(SessionOptions o, CancellationToken ct = default)
    {
        var links = new List<IDroneLink>();
        for (int i = 0; i < o.DroneCount; i++)
        {
            links.Add(o.Mode switch
            {
                LinkMode.Simulation => new SimDroneLink(i + 1, GeoMath.Offset(o.Origin, -(i / 5) * 6, (i % 5 - 2) * 6, 0), headingDeg: 0),
                LinkMode.SitlTcp => new MavlinkDroneLink(i + 1, new TcpMavTransport(o.Host, o.BasePort + o.PortStride * i)),
                _ => new MavlinkDroneLink(i + 1, new UdpMavTransport(o.BasePort + o.PortStride * i)),
            });
        }
        var s = new SwarmSession(o, links);
        s.Write(LogLevel.Info, "SkyHive", o.Mode == LinkMode.Simulation
            ? $"Dahili simülasyon: {o.DroneCount} drone"
            : $"ArduPilot SITL: {o.DroneCount} araç, {o.Host}:{o.BasePort}+{o.PortStride}·n ({o.Mode})");
        foreach (var l in links) await l.StartAsync(s._cts.Token);
        s._loops.Add(Task.Run(() => s.Controller.RunAsync(() => s.Origin, s._cts.Token)));
        s._loops.Add(Task.Run(() => s.Vision.RunAsync(s._cts.Token)));
        s._loops.Add(Task.Run(() => s.WorldLoopAsync(s._cts.Token)));
        return s;
    }

    async Task WorldLoopAsync(CancellationToken ct)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(100));
        while (await timer.WaitForNextTickAsync(ct).ConfigureAwait(false))
        {
            World.Step(0.1);
            // SITL with a custom home: re-centre the world on the first vehicle's home
            if (Options.Mode != LinkMode.Simulation && Agents[0].State.Home is { } home && GeoMath.Distance(home, Origin) > 1500)
            {
                Origin = home;
                foreach (var a in Agents) a.ClearTrail();
                Write(LogLevel.Info, "SkyHive", $"Harita merkezi D1 evine taşındı: {home}");
            }
        }
    }

    /// <summary>Scripted showcase: takeoff → wedge formation sweep → split area search.</summary>
    public async Task RunDemoAsync(CancellationToken ct = default)
    {
        Write(LogLevel.Info, "Demo", "Senaryo başlıyor: kalkış → formasyon → alan taraması");
        await Controller.SetSpeedAsync(null, 8);
        await Controller.TakeoffAsync(null, 22);
        await WaitUntil(() => Agents.All(a => a.Alt > 18 || !a.State.Connected), TimeSpan.FromSeconds(90), ct);
        Controller.Formation(FormationType.Wedge, 12, Agents[0].Id);
        Controller.Goto(Agents[0].Id, new Vec2(120, 60), 25);
        await WaitUntil(() => Agents[0].Local.DistanceTo(new Vec2(120, 60)) < 4, TimeSpan.FromSeconds(90), ct);
        var dist = Controller.Search(null, new Vec2(40, 40), 320, 260, 24, Vision.Camera);
        Write(LogLevel.Info, "Demo", $"Alan taraması başladı: toplam rota {dist:F0} m");
    }

    static async Task WaitUntil(Func<bool> cond, TimeSpan timeout, CancellationToken ct)
    {
        var end = DateTime.UtcNow + timeout;
        while (!cond() && DateTime.UtcNow < end) await Task.Delay(250, ct);
    }

    public async ValueTask DisposeAsync()
    {
        _cts.Cancel();
        foreach (var t in _loops) { try { await t; } catch { } }
        foreach (var a in Agents) await a.Link.DisposeAsync();
    }
}
