using SkyHive.Core.Geo;
using SkyHive.Core.Model;
using SkyHive.Core.Vision;

namespace SkyHive.Core.Swarm;

/// <summary>
/// Central swarm brain: owns per-drone tasks and streams guided targets to every vehicle at 4 Hz.
/// Also performs altitude layering and a simple proximity (separation) guard.
/// </summary>
public sealed class SwarmController
{
    readonly List<DroneAgent> _agents;
    readonly TargetTracker _tracker;
    readonly Dictionary<(int, int), DateTime> _proximityWarned = new();
    readonly DateTime _start = DateTime.UtcNow;

    public SwarmController(IEnumerable<DroneAgent> agents, TargetTracker tracker)
    {
        _agents = agents.ToList();
        _tracker = tracker;
    }

    public IReadOnlyList<DroneAgent> Agents => _agents;
    public double LayerStep { get; set; } = 1.5;
    public double MinSeparation { get; set; } = 3.0;
    public event Action<LogLevel, string, string>? Log;

    void Say(LogLevel l, string src, string msg) => Log?.Invoke(l, src, msg);

    public DroneAgent? Find(int id) => _agents.FirstOrDefault(a => a.Id == id);

    IEnumerable<DroneAgent> Select(IEnumerable<int>? ids) => ids is null ? _agents : _agents.Where(a => ids.Contains(a.Id));

    public Vec2 Centroid(IEnumerable<int>? ids = null)
    {
        var list = Select(ids).Where(a => a.State.HasPosition).ToList();
        if (list.Count == 0) return Vec2.Zero;
        return new Vec2(list.Average(a => a.Local.N), list.Average(a => a.Local.E));
    }

    // ------------------------------------------------------------------ loop

    public async Task RunAsync(Func<GeoPoint> origin, CancellationToken ct)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(250));
        while (await timer.WaitForNextTickAsync(ct).ConfigureAwait(false))
        {
            try { await TickAsync(origin()); }
            catch (Exception ex) { Say(LogLevel.Error, "Sürü", ex.Message); }
        }
    }

    public async Task TickAsync(GeoPoint origin)
    {
        var ctx = new SwarmContext { Agents = _agents, Tracker = _tracker, Time = (DateTime.UtcNow - _start).TotalSeconds };
        var sends = new List<Task>();
        foreach (var a in _agents)
        {
            if (!a.State.HasPosition) continue;
            a.RecordTrail();
            var task = a.Task;
            if (task is null) continue;
            if (!a.State.Armed) { a.Task = null; continue; }
            if (a.State.Mode is FlightMode.Land or FlightMode.Rtl)
            {
                // failsafe or pilot RTL/LAND overrides the swarm task; freshly assigned tasks wait for the GUIDED switch
                if ((DateTime.UtcNow - a.TaskAssigned).TotalSeconds > 3) a.Task = null;
                continue;
            }
            if (a.Alt < 2.5) continue; // let the takeoff finish first

            var next = task.Next(a, ctx);
            if (task.Completed)
            {
                Say(LogLevel.Success, a.Name, $"{task.Name} tamamlandı");
                a.Task = next is { } n ? new HoldTask(n.Position, n.Alt) : null;
            }
            if (next is null) continue;
            var target = next.Value;

            // separation guard: climb above any drone that is too close
            foreach (var other in _agents)
            {
                if (other.Id == a.Id || !other.State.Airborne) continue;
                var dh = other.Local.DistanceTo(a.Local);
                var dv = Math.Abs(other.Alt - a.Alt);
                if (dh < MinSeparation && dv < 2.0 && a.Index > other.Index)
                {
                    target = target with { Alt = Math.Max(target.Alt, other.Alt + 3) };
                    var key = (other.Id, a.Id);
                    if (!_proximityWarned.TryGetValue(key, out var w) || (DateTime.UtcNow - w).TotalSeconds > 8)
                    {
                        _proximityWarned[key] = DateTime.UtcNow;
                        Say(LogLevel.Warning, a.Name, $"{other.Name} ile yakınlık {dh:F1} m — irtifa ayrımı uygulanıyor");
                    }
                }
            }

            var changed = a.LastTarget is not { } last || last.Position.DistanceTo(target.Position) > 0.7 || Math.Abs(last.Alt - target.Alt) > 0.4;
            if (changed || (DateTime.UtcNow - a.LastSent).TotalSeconds > 2)
            {
                a.LastTarget = target;
                a.LastSent = DateTime.UtcNow;
                sends.Add(a.Link.GotoAsync(GeoMath.FromLocal(origin, target.Position, target.Alt)));
            }
        }
        await Task.WhenAll(sends);
    }

    // ------------------------------------------------------------- commands

    /// <summary>Gives a drone a new task; pulls it out of RTL/LAND back into GUIDED if needed.</summary>
    void Assign(DroneAgent a, SwarmTask task)
    {
        a.Task = task;
        a.LastTarget = null;
        if (a.State.Airborne && a.State.Mode is FlightMode.Rtl or FlightMode.Land)
            _ = a.Link.SetModeAsync(FlightMode.Guided);
    }

    public async Task<int> ArmAsync(IEnumerable<int>? ids, bool arm)
    {
        var res = await Task.WhenAll(Select(ids).Select(a => a.Link.ArmAsync(arm)));
        return res.Count(r => r);
    }

    public async Task<int> TakeoffAsync(IEnumerable<int>? ids, double alt)
    {
        var list = Select(ids).ToList();
        foreach (var a in list) { a.Task = null; a.LastTarget = null; }
        var res = await Task.WhenAll(list.Select(a => a.Link.TakeoffAsync(alt)));
        return res.Count(r => r);
    }

    public async Task LandAsync(IEnumerable<int>? ids)
    {
        var list = Select(ids).ToList();
        foreach (var a in list) a.Task = null;
        await Task.WhenAll(list.Select(a => a.Link.LandAsync()));
    }

    public async Task RtlAsync(IEnumerable<int>? ids)
    {
        var list = Select(ids).ToList();
        foreach (var a in list) a.Task = null;
        await Task.WhenAll(list.Select(a => a.Link.ReturnToLaunchAsync()));
    }

    public void Hold(IEnumerable<int>? ids)
    {
        foreach (var a in Select(ids).Where(a => a.State.Airborne))
            Assign(a, new HoldTask(a.Local, a.Alt));
    }

    public async Task SetSpeedAsync(IEnumerable<int>? ids, double speed) =>
        await Task.WhenAll(Select(ids).Select(a => a.Link.SetSpeedAsync(speed)));

    public void Goto(int id, Vec2 point, double? alt = null)
    {
        var a = Find(id) ?? throw new ArgumentException($"D{id} yok");
        Assign(a, new GotoTask(point, alt ?? Math.Max(a.Alt, 10)));
    }

    public int Formation(FormationType type, double spacing, int leaderId, IEnumerable<int>? ids = null)
    {
        var leader = Find(leaderId) ?? throw new ArgumentException($"Lider D{leaderId} yok");
        var followers = Select(ids).Where(a => a.Id != leaderId).ToList();
        var slots = Formations.Slots(type, followers.Count + 1, spacing);
        if (leader.Task is FormationTask) leader.Task = null;
        if (leader.Task is null && leader.State.Airborne) Assign(leader, new HoldTask(leader.Local, leader.Alt));
        for (int k = 0; k < followers.Count; k++)
        {
            var (f, r) = slots[k + 1];
            Assign(followers[k], new FormationTask(leaderId, type, f, r, ((k % 3) - 1) * LayerStep));
        }
        return followers.Count;
    }

    public int Orbit(IEnumerable<int>? ids, Vec2 center, double radius, double alt, double speed = 5)
    {
        var list = Select(ids).ToList();
        for (int i = 0; i < list.Count; i++)
            Assign(list[i], new OrbitTask(() => center, radius, alt + i * LayerStep, speed, 2 * Math.PI * i / list.Count));
        return list.Count;
    }

    public double Search(IEnumerable<int>? ids, Vec2 center, double width, double height, double alt, CameraModel cam)
    {
        var list = Select(ids).ToList();
        if (list.Count == 0) return 0;
        var lane = cam.FootprintWidth(alt) * 0.75;
        var paths = PathPlanner.LawnmowerSplit(center, width, height, list.Count, lane);
        double total = 0;
        for (int i = 0; i < list.Count; i++)
        {
            Assign(list[i], new WaypointTask("Alan taraması", paths[i], alt + i * 0.5 * LayerStep, loop: false));
            total += PathPlanner.PathLength(paths[i]);
        }
        return total;
    }

    public int Patrol(IEnumerable<int>? ids, Vec2 center, double size, double alt)
    {
        var list = Select(ids).ToList();
        var square = PathPlanner.Square(center, size);
        for (int i = 0; i < list.Count; i++)
        {
            int shift = i % square.Count;
            var wps = square.Skip(shift).Concat(square.Take(shift)).ToList();
            Assign(list[i], new WaypointTask("Devriye", wps, alt + i * LayerStep, loop: true));
        }
        return list.Count;
    }

    public DroneAgent? Track(int targetId, IEnumerable<int>? ids = null, double radius = 14, double alt = 18)
    {
        var target = _tracker.Find(targetId) ?? throw new ArgumentException($"T{targetId} bulunamadı");
        var candidates = Select(ids).Where(a => a.State.Airborne).ToList();
        if (candidates.Count == 0) return null;
        var drone = candidates.OrderBy(a => a.Local.DistanceTo(target.Position)).First();
        Assign(drone, new OrbitTask(() => _tracker.Find(targetId) is { Stale: false } t ? t.Position + t.Velocity * 1.0 : null,
            radius, alt, 6, 0, $"Takip {target.Label}"));
        return drone;
    }
}
