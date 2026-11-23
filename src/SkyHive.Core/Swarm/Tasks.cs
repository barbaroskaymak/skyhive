using SkyHive.Core.Geo;
using SkyHive.Core.Vision;

namespace SkyHive.Core.Swarm;

/// <summary>Context a task needs to compute the next guided target.</summary>
public sealed class SwarmContext
{
    public required IReadOnlyList<DroneAgent> Agents { get; init; }
    public required TargetTracker Tracker { get; init; }
    public double Time { get; init; }
    public DroneAgent? Find(int id) => Agents.FirstOrDefault(a => a.Id == id);
}

public readonly record struct Target3(Vec2 Position, double Alt);

/// <summary>Behaviour assigned to a drone. Evaluated by the controller at ~4 Hz.</summary>
public abstract class SwarmTask
{
    public abstract string Name { get; }
    public bool Completed { get; protected set; }
    public abstract Target3? Next(DroneAgent agent, SwarmContext ctx);
    /// <summary>Remaining path for map drawing.</summary>
    public virtual IReadOnlyList<Vec2> PlannedPath => Array.Empty<Vec2>();
}

public sealed class GotoTask(Vec2 point, double alt) : SwarmTask
{
    public override string Name => "Hedefe git";
    public Vec2 Point { get; } = point;
    public override Target3? Next(DroneAgent a, SwarmContext ctx)
    {
        if (a.Local.DistanceTo(Point) < 1.5 && Math.Abs(a.Alt - alt) < 1.0) Completed = true;
        return new(Point, alt);
    }
    public override IReadOnlyList<Vec2> PlannedPath => new[] { Point };
}

public sealed class HoldTask(Vec2 point, double alt) : SwarmTask
{
    public override string Name => "Askıda bekle";
    public override Target3? Next(DroneAgent a, SwarmContext ctx) => new(point, alt);
}

/// <summary>Waypoint mission (optionally looping) flown in GUIDED mode.</summary>
public sealed class WaypointTask(string name, IReadOnlyList<Vec2> waypoints, double alt, bool loop, double acceptRadius = 2.5) : SwarmTask
{
    int _index;
    public override string Name => loop ? $"{name} (döngü)" : $"{name} {Math.Min(_index + 1, waypoints.Count)}/{waypoints.Count}";
    public int Index => _index;
    public double Progress => waypoints.Count == 0 ? 1 : (double)_index / waypoints.Count;
    public override Target3? Next(DroneAgent a, SwarmContext ctx)
    {
        if (waypoints.Count == 0) { Completed = true; return null; }
        if (a.Local.DistanceTo(waypoints[_index]) < acceptRadius)
        {
            _index++;
            if (_index >= waypoints.Count)
            {
                if (loop) _index = 0;
                else { Completed = true; _index = waypoints.Count - 1; }
            }
        }
        return new(waypoints[_index], alt);
    }
    public override IReadOnlyList<Vec2> PlannedPath => waypoints.Skip(_index).ToList();
}

/// <summary>Follows a leader drone at a body-frame slot offset.</summary>
public sealed class FormationTask(int leaderId, FormationType type, double forward, double right, double altOffset) : SwarmTask
{
    double _formationHeading = double.NaN;
    public override string Name => $"Formasyon · {Formations.DisplayName(type)}";
    public int LeaderId => leaderId;
    public override Target3? Next(DroneAgent a, SwarmContext ctx)
    {
        var leader = ctx.Find(leaderId);
        if (leader is null || !leader.State.Airborne) return null;
        if (double.IsNaN(_formationHeading) || leader.State.GroundSpeed > 1.2) _formationHeading = leader.State.Heading;
        var p = leader.Local + Vec2.FromBody(forward, right, _formationHeading);
        // lead the moving slot a little so followers keep up
        var leadVec = Vec2.FromPolar(leader.State.Heading, Math.Min(leader.State.GroundSpeed * 1.2, 8));
        return new(p + leadVec, Math.Max(3, leader.Alt + altOffset));
    }
}

/// <summary>Several drones evenly spaced on a rotating ring around a point.</summary>
public sealed class OrbitTask(Func<Vec2?> center, double radius, double alt, double speed, double phase, string label = "Yörünge") : SwarmTask
{
    double _theta = double.NaN;
    double _lastT;
    public override string Name => $"{label} r={radius:F0} m";
    public double Radius => radius;
    public Vec2? Center => center();
    public override Target3? Next(DroneAgent a, SwarmContext ctx)
    {
        var c = center();
        if (c is null) { Completed = true; return null; }
        if (double.IsNaN(_theta)) { _theta = phase; _lastT = ctx.Time; }
        _theta += (ctx.Time - _lastT) * speed / radius;
        _lastT = ctx.Time;
        var ahead = _theta + Math.Min(0.6, 10 / radius);
        return new(c.Value + new Vec2(Math.Cos(ahead) * radius, Math.Sin(ahead) * radius), alt);
    }
}
