using SkyHive.Core.Geo;
using SkyHive.Core.Links;
using SkyHive.Core.Model;

namespace SkyHive.Core.Swarm;

/// <summary>A drone in the swarm: its link plus controller-side state (task, trail, local position).</summary>
public sealed class DroneAgent(IDroneLink link, int index, Func<GeoPoint> origin)
{
    readonly object _trailGate = new();
    readonly LinkedList<Vec2> _trail = new();

    public IDroneLink Link { get; } = link;
    public int Index { get; } = index;
    public int Id => Link.Id;
    public string Name => Link.Name;
    public DroneState State => Link.State;
    SwarmTask? _task;
    public SwarmTask? Task
    {
        get => _task;
        set { _task = value; TaskAssigned = DateTime.UtcNow; }
    }
    public DateTime TaskAssigned { get; private set; }
    public Target3? LastTarget { get; set; }
    public DateTime LastSent { get; set; }
    public Vec2 Local => GeoMath.ToLocal(origin(), State.Position);
    public double Alt => State.Position.Alt;
    public string TaskName => Task?.Name ?? (State.Airborne ? "Bekliyor" : State.Armed ? "Yerde (armed)" : "Yerde");

    public void RecordTrail()
    {
        lock (_trailGate)
        {
            var p = Local;
            if (_trail.Last is { } last && last.Value.DistanceTo(p) < 1.5) return;
            _trail.AddLast(p);
            while (_trail.Count > 240) _trail.RemoveFirst();
        }
    }

    public IReadOnlyList<Vec2> Trail()
    {
        lock (_trailGate) return _trail.ToList();
    }

    public void ClearTrail()
    {
        lock (_trailGate) _trail.Clear();
    }
}
