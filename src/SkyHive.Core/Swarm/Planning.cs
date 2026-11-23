using SkyHive.Core.Geo;

namespace SkyHive.Core.Swarm;

public enum FormationType { Line, Column, Wedge, Circle, Grid }

public static class Formations
{
    /// <summary>Body-frame slot offsets (forward, right) for <paramref name="count"/> drones; slot 0 is the leader at (0,0).</summary>
    public static IReadOnlyList<(double Forward, double Right)> Slots(FormationType type, int count, double spacing)
    {
        var slots = new List<(double, double)> { (0, 0) };
        switch (type)
        {
            case FormationType.Line:
                for (int k = 1; k < count; k++) slots.Add((0, spacing * ((k + 1) / 2) * (k % 2 == 1 ? 1 : -1)));
                break;
            case FormationType.Column:
                for (int k = 1; k < count; k++) slots.Add((-spacing * k, 0));
                break;
            case FormationType.Wedge:
                for (int k = 1; k < count; k++)
                {
                    int rank = (k + 1) / 2;
                    slots.Add((-spacing * rank, spacing * rank * (k % 2 == 1 ? 1 : -1)));
                }
                break;
            case FormationType.Circle:
                int ring = count - 1;
                if (ring > 0)
                {
                    double r = ring == 1 ? spacing : Math.Max(spacing, spacing / (2 * Math.Sin(Math.PI / ring)));
                    for (int k = 0; k < ring; k++)
                    {
                        double a = 2 * Math.PI * k / ring;
                        slots.Add((r * Math.Cos(a), r * Math.Sin(a)));
                    }
                }
                break;
            case FormationType.Grid:
                int cols = (int)Math.Ceiling(Math.Sqrt(count));
                var all = new List<(double, double)>();
                for (int i = 0; i < count; i++)
                    all.Add((-(i / cols) * spacing, (i % cols - (cols - 1) / 2.0) * spacing));
                var lead = all[0];
                return all.Select(s => (s.Item1 - lead.Item1, s.Item2 - lead.Item2)).ToList();
        }
        return slots;
    }

    public static string DisplayName(FormationType t) => t switch
    {
        FormationType.Line => "Hat",
        FormationType.Column => "Kol",
        FormationType.Wedge => "V (Kama)",
        FormationType.Circle => "Çember",
        FormationType.Grid => "Izgara",
        _ => t.ToString(),
    };
}

public static class PathPlanner
{
    /// <summary>
    /// Splits a rectangular area (centre, width along east, height along north) into one strip per drone
    /// and builds a boustrophedon ("lawnmower") path for each strip.
    /// </summary>
    public static List<List<Vec2>> LawnmowerSplit(Vec2 center, double width, double height, int drones, double laneSpacing)
    {
        var result = new List<List<Vec2>>();
        double stripW = width / drones;
        double south = center.N - height / 2, north = center.N + height / 2;
        for (int d = 0; d < drones; d++)
        {
            double left = center.E - width / 2 + d * stripW;
            int lanes = Math.Max(1, (int)Math.Ceiling(stripW / laneSpacing));
            double step = stripW / lanes;
            var path = new List<Vec2>();
            for (int l = 0; l < lanes; l++)
            {
                double e = left + step * (l + 0.5);
                bool up = l % 2 == 0;
                path.Add(new Vec2(up ? south : north, e));
                path.Add(new Vec2(up ? north : south, e));
            }
            result.Add(path);
        }
        return result;
    }

    public static List<Vec2> Square(Vec2 center, double size) => new()
    {
        center + new Vec2(size / 2, -size / 2), center + new Vec2(size / 2, size / 2),
        center + new Vec2(-size / 2, size / 2), center + new Vec2(-size / 2, -size / 2),
    };

    public static double PathLength(IReadOnlyList<Vec2> path)
    {
        double len = 0;
        for (int i = 1; i < path.Count; i++) len += path[i - 1].DistanceTo(path[i]);
        return len;
    }
}
