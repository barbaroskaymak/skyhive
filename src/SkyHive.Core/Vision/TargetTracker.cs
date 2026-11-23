using SkyHive.Core.Geo;

namespace SkyHive.Core.Vision;

public sealed class TrackedTarget
{
    public int Id { get; init; }
    public string Label => $"T{Id}";
    public Vec2 Position { get; set; }
    public Vec2 Velocity { get; set; }
    public DateTime FirstSeen { get; init; }
    public DateTime LastSeen { get; set; }
    public int Hits { get; set; }
    public double Confidence { get; set; }
    public double ApproxSizeM { get; set; }
    public string LastSeenBy { get; set; } = "";
    internal Vec2 AnchorPosition { get; set; }
    internal DateTime AnchorTime { get; set; }
    public bool Confirmed => Hits >= 3;
    public bool Stale => (DateTime.UtcNow - LastSeen).TotalSeconds > 90;
}

/// <summary>Fuses geo-referenced detections from every drone camera into one shared target list.</summary>
public sealed class TargetTracker
{
    readonly object _gate = new();
    readonly List<TrackedTarget> _tracks = new();
    int _nextId = 1;
    public double AssociationRadius { get; set; } = 7;
    public event Action<TrackedTarget>? TargetConfirmed;

    public IReadOnlyList<TrackedTarget> Snapshot()
    {
        lock (_gate) return _tracks.Where(t => !t.Stale).OrderBy(t => t.Id).ToList();
    }

    public TrackedTarget? Find(int id)
    {
        lock (_gate) return _tracks.FirstOrDefault(t => t.Id == id);
    }

    public void Report(Vec2 position, double confidence, double sizeM, string droneName, DateTime? now = null)
    {
        var t0 = now ?? DateTime.UtcNow;
        TrackedTarget? confirmed = null;
        lock (_gate)
        {
            TrackedTarget? best = null;
            double bestD = double.MaxValue;
            foreach (var t in _tracks)
            {
                var predicted = t.Position + t.Velocity * (t0 - t.LastSeen).TotalSeconds;
                var d = predicted.DistanceTo(position);
                if (d < AssociationRadius + Math.Min(10, t.Velocity.Length * 2) && d < bestD) { best = t; bestD = d; }
            }
            if (best is null)
            {
                _tracks.Add(new TrackedTarget
                {
                    Id = _nextId++, Position = position, FirstSeen = t0, LastSeen = t0, Hits = 1,
                    Confidence = confidence, ApproxSizeM = sizeM, LastSeenBy = droneName,
                    AnchorPosition = position, AnchorTime = t0,
                });
                return;
            }
            var gap = (t0 - best.LastSeen).TotalSeconds;
            best.Position = best.Position * 0.5 + position * 0.5;
            // velocity from a >=1.5 s baseline so per-frame georeferencing noise does not look like motion
            var baseline = (t0 - best.AnchorTime).TotalSeconds;
            if (gap > 6) { best.Velocity = Vec2.Zero; best.AnchorPosition = best.Position; best.AnchorTime = t0; }
            else if (baseline >= 1.5)
            {
                var v = (best.Position - best.AnchorPosition) / baseline;
                if (v.Length > 15) v = v.Normalized * 15;
                best.Velocity = best.Velocity * 0.5 + v * 0.5;
                if (best.Velocity.Length < 0.6) best.Velocity = Vec2.Zero;
                best.AnchorPosition = best.Position;
                best.AnchorTime = t0;
            }
            best.LastSeen = t0;
            best.Hits++;
            best.Confidence = Math.Max(best.Confidence * 0.9, confidence);
            best.ApproxSizeM = best.ApproxSizeM * 0.7 + sizeM * 0.3;
            best.LastSeenBy = droneName;
            if (best.Hits == 3) confirmed = best;
        }
        if (confirmed is not null) TargetConfirmed?.Invoke(confirmed);
    }

    public void Clear()
    {
        lock (_gate) { _tracks.Clear(); _nextId = 1; }
    }
}
