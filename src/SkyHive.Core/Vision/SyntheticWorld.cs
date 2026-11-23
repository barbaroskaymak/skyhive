using SkyHive.Core.Geo;

namespace SkyHive.Core.Vision;

public enum TargetKind { Vehicle, Person, Container }

/// <summary>A ground-truth object in the synthetic world (only visible to the swarm through cameras).</summary>
public sealed class WorldTarget
{
    public int Id { get; init; }
    public TargetKind Kind { get; init; }
    public Vec2 Position { get; set; }
    public double Heading { get; set; }
    public double Length { get; init; }
    public double Width { get; init; }
    public double Speed { get; init; }
    public int RoadIndex { get; init; } = -1;
    public double RoadT { get; set; }
    public int Direction { get; set; } = 1;
}

/// <summary>
/// Procedural terrain (fields, roads, lake, buildings) around the home position with red targets placed on it.
/// Used to synthesise downward camera frames for both the built-in simulator and SITL vehicles (SITL has no camera).
/// </summary>
public sealed class SyntheticWorld
{
    public const double Extent = 600; // ± metres around origin
    readonly int _seed;
    readonly object _gate = new();
    readonly List<WorldTarget> _targets = new();
    public IReadOnlyList<(Vec2 A, Vec2 B)> Roads { get; }

    public SyntheticWorld(int seed = 1337, int targetCount = 14)
    {
        _seed = seed;
        Roads = new List<(Vec2, Vec2)>
        {
            (new(-600, -180), new(600, 120)),
            (new(-260, -600), new(220, 600)),
            (new(320, -600), new(380, 600)),
            (new(-420, 200), new(80, 560)),
        };
        var rng = new Random(seed);
        for (int i = 0; i < targetCount; i++)
        {
            bool moving = i % 4 == 0;
            var kind = (TargetKind)(i % 3);
            var t = new WorldTarget
            {
                Id = i + 1,
                Kind = kind,
                Length = kind switch { TargetKind.Vehicle => 4.6, TargetKind.Person => 1.2, _ => 6.0 },
                Width = kind switch { TargetKind.Vehicle => 2.0, TargetKind.Person => 1.2, _ => 2.5 },
                Speed = moving ? 2.5 + rng.NextDouble() * 4 : 0,
                RoadIndex = moving ? i % Roads.Count : -1,
                RoadT = rng.NextDouble(),
                Heading = rng.NextDouble() * 360,
            };
            t.Position = moving ? RoadPoint(t.RoadIndex, t.RoadT) : new Vec2((rng.NextDouble() - 0.5) * 700, (rng.NextDouble() - 0.5) * 700);
            if (moving) t.Heading = (Roads[t.RoadIndex].B - Roads[t.RoadIndex].A).Bearing;
            _targets.Add(t);
        }
    }

    public IReadOnlyList<WorldTarget> SnapshotTargets()
    {
        lock (_gate) return _targets.Select(t => new WorldTarget
        {
            Id = t.Id, Kind = t.Kind, Position = t.Position, Heading = t.Heading, Length = t.Length, Width = t.Width, Speed = t.Speed,
        }).ToList();
    }

    Vec2 RoadPoint(int road, double t)
    {
        var (a, b) = Roads[road];
        return a + (b - a) * t;
    }

    /// <summary>Moves road-bound targets.</summary>
    public void Step(double dt)
    {
        lock (_gate)
        {
            foreach (var t in _targets.Where(t => t.RoadIndex >= 0))
            {
                var (a, b) = Roads[t.RoadIndex];
                var len = (b - a).Length;
                t.RoadT += t.Direction * t.Speed * dt / len;
                if (t.RoadT > 0.95 || t.RoadT < 0.05) { t.Direction = -t.Direction; t.RoadT = Math.Clamp(t.RoadT, 0.05, 0.95); }
                // drive on the right-hand lane
                var dir = (b - a).Normalized * t.Direction;
                var right = new Vec2(-dir.E, dir.N) * -1.6;
                t.Position = RoadPoint(t.RoadIndex, t.RoadT) + new Vec2(right.N, right.E);
                t.Heading = dir.Bearing;
            }
        }
    }

    // ---------------- terrain ----------------

    static uint Hash(int x, int y, int seed)
    {
        unchecked
        {
            uint h = (uint)(x * 374761393 + y * 668265263 + seed * 1442695041);
            h = (h ^ (h >> 13)) * 1274126177;
            return h ^ (h >> 16);
        }
    }

    static double Rand01(int x, int y, int seed) => (Hash(x, y, seed) & 0xFFFFFF) / (double)0xFFFFFF;

    double ValueNoise(double x, double y, int salt)
    {
        int xi = (int)Math.Floor(x), yi = (int)Math.Floor(y);
        double fx = x - xi, fy = y - yi;
        fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy);
        double a = Rand01(xi, yi, _seed + salt), b = Rand01(xi + 1, yi, _seed + salt);
        double c = Rand01(xi, yi + 1, _seed + salt), d = Rand01(xi + 1, yi + 1, _seed + salt);
        return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy;
    }

    double Fbm(double x, double y, int salt, int octaves = 3)
    {
        double sum = 0, amp = 0.5, f = 1;
        for (int i = 0; i < octaves; i++) { sum += amp * ValueNoise(x * f, y * f, salt + i * 31); amp *= 0.5; f *= 2.03; }
        return sum / (1 - Math.Pow(0.5, octaves));
    }

    static double DistToSegment(Vec2 p, Vec2 a, Vec2 b, out double t)
    {
        var ab = b - a;
        t = Math.Clamp(((p.N - a.N) * ab.N + (p.E - a.E) * ab.E) / (ab.N * ab.N + ab.E * ab.E), 0, 1);
        return p.DistanceTo(a + ab * t);
    }

    /// <summary>Terrain colour at a local point, without targets.</summary>
    public (byte R, byte G, byte B) SampleTerrain(Vec2 p, double detail = 1.0)
    {
        double n = p.N, e = p.E;
        // lake
        double lake = Fbm(n / 180 + 3.1, e / 180 - 7.7, 900, 3);
        double lakeDist = new Vec2(n + 210, e - 230).Length;
        double water = lake * 0.6 + lakeDist / 260;
        if (water < 0.62)
        {
            double depth = Math.Clamp((0.62 - water) * 6, 0, 1);
            double ripple = detail > 0.5 ? (ValueNoise(n / 3, e / 3, 77) - 0.5) * 10 : 0;
            return Px(40 - 20 * depth + ripple, 92 - 30 * depth + ripple, 128 + 10 * depth + ripple);
        }
        if (water < 0.645) return Px(176, 166, 128); // shore sand

        // roads
        for (int i = 0; i < Roads.Count; i++)
        {
            var d = DistToSegment(p, Roads[i].A, Roads[i].B, out var t);
            if (d < 3.6)
            {
                double along = t * (Roads[i].B - Roads[i].A).Length;
                bool dash = d < 0.18 && (along % 6) < 3;
                if (dash) return Px(230, 228, 210);
                double g = 78 + (ValueNoise(n / 1.5, e / 1.5, 5) - 0.5) * (detail > 0.5 ? 14 : 0);
                return Px(g, g + 2, g + 6);
            }
            if (d < 4.6) return Px(128, 118, 96); // gravel shoulder
        }

        // field parcels on a warped 45 m grid
        double wn = n + (Fbm(n / 90, e / 90, 300) - 0.5) * 30;
        double we = e + (Fbm(n / 90 + 9, e / 90 - 4, 301) - 0.5) * 30;
        int cx = (int)Math.Floor(wn / 45), cy = (int)Math.Floor(we / 45);
        double lx = wn / 45 - cx, ly = we / 45 - cy;
        double edge = Math.Min(Math.Min(lx, 1 - lx), Math.Min(ly, 1 - ly)) * 45;
        double kind = Rand01(cx, cy, _seed + 11);

        // buildings in some parcels
        if (kind > 0.9)
        {
            double bx = Math.Abs(lx - 0.5), by = Math.Abs(ly - 0.5);
            if (bx < 0.22 && by < 0.16)
            {
                double roof = Rand01(cx, cy, _seed + 12);
                double ridge = Math.Abs(by - 0.0) < 0.012 ? -25 : 0;
                return roof > 0.5 ? Px(205 + ridge, 205 + ridge, 200 + ridge) : Px(120 + ridge, 128 + ridge, 140 + ridge);
            }
            if (bx < 0.25 && by < 0.2 && lx > 0.5 && ly > 0.5) return Px(40, 46, 38); // shadow
            return Px(150, 150, 140); // yard
        }

        if (edge < 1.2) return Px(52, 74, 40); // hedge rows

        double tex = Fbm(n / 14, e / 14, 400, detail > 0.5 ? 3 : 2);
        double rows = detail > 0.5 ? Math.Sin((kind > 0.5 ? wn : we) * 2.2) * 0.5 + 0.5 : 0.5;
        double v = 0.85 + tex * 0.3;
        if (kind < 0.40) return Px((62 + rows * 10) * v, (110 + rows * 14) * v, (48) * v);          // green crops
        if (kind < 0.62) return Px((150 + rows * 12) * v, (150 + rows * 10) * v, (78) * v);         // wheat
        if (kind < 0.78) return Px((112) * v, (92 + rows * 8) * v, (66) * v);                      // ploughed soil
        if (kind < 0.90)                                                                             // forest patches
        {
            double canopy = Fbm(n / 4, e / 4, 500, 3);
            return Px(34 + canopy * 30, 70 + canopy * 40, 34 + canopy * 18);
        }
        return Px(90 * v, 128 * v, 70 * v);
    }

    static (byte, byte, byte) Px(double r, double g, double b) =>
        ((byte)Math.Clamp(r, 0, 255), (byte)Math.Clamp(g, 0, 255), (byte)Math.Clamp(b, 0, 255));

    /// <summary>Colour of a target pixel if <paramref name="p"/> falls on one.</summary>
    public static (byte R, byte G, byte B)? SampleTargets(Vec2 p, IReadOnlyList<WorldTarget> targets)
    {
        foreach (var t in targets)
        {
            var d = p - t.Position;
            if (Math.Abs(d.N) > 4 || Math.Abs(d.E) > 4) continue;
            var h = t.Heading * Math.PI / 180;
            double fwd = d.N * Math.Cos(h) + d.E * Math.Sin(h);
            double right = -d.N * Math.Sin(h) + d.E * Math.Cos(h);
            if (t.Kind == TargetKind.Person)
            {
                var r = Math.Sqrt(fwd * fwd + right * right);
                if (r < 0.6) return r < 0.3 ? ((byte)240, (byte)60, (byte)50) : ((byte)200, (byte)30, (byte)30);
                continue;
            }
            if (Math.Abs(fwd) < t.Length / 2 && Math.Abs(right) < t.Width / 2)
            {
                bool windshield = t.Kind == TargetKind.Vehicle && fwd > t.Length * 0.12 && fwd < t.Length * 0.3 && Math.Abs(right) < t.Width * 0.4;
                if (windshield) return (30, 40, 60);
                bool edge = Math.Abs(fwd) > t.Length / 2 - 0.25 || Math.Abs(right) > t.Width / 2 - 0.25;
                return edge ? ((byte)150, (byte)20, (byte)24) : ((byte)214, (byte)36, (byte)36);
            }
        }
        return null;
    }
}
