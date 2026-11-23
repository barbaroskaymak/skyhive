namespace SkyHive.Core.Geo;

/// <summary>WGS84 position. <see cref="Alt"/> is metres above home (relative altitude).</summary>
public readonly record struct GeoPoint(double Lat, double Lon, double Alt)
{
    public override string ToString() => $"{Lat:F6}, {Lon:F6} @ {Alt:F1} m";
}

/// <summary>Local tangent-plane vector in metres: North / East.</summary>
public readonly record struct Vec2(double N, double E)
{
    public static readonly Vec2 Zero = new(0, 0);
    public double Length => Math.Sqrt(N * N + E * E);
    public Vec2 Normalized => Length < 1e-9 ? Zero : this / Length;
    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.N + b.N, a.E + b.E);
    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.N - b.N, a.E - b.E);
    public static Vec2 operator *(Vec2 a, double k) => new(a.N * k, a.E * k);
    public static Vec2 operator /(Vec2 a, double k) => new(a.N / k, a.E / k);
    public double DistanceTo(Vec2 o) => (this - o).Length;

    /// <summary>Bearing in degrees (0 = north, clockwise).</summary>
    public double Bearing => GeoMath.NormalizeDeg(Math.Atan2(E, N) * 180 / Math.PI);

    /// <summary>Builds a vector from a body-frame offset (forward/right) rotated by a heading.</summary>
    public static Vec2 FromBody(double forward, double right, double headingDeg)
    {
        var h = headingDeg * Math.PI / 180;
        return new(forward * Math.Cos(h) - right * Math.Sin(h), forward * Math.Sin(h) + right * Math.Cos(h));
    }

    public static Vec2 FromPolar(double bearingDeg, double distance)
    {
        var h = bearingDeg * Math.PI / 180;
        return new(Math.Cos(h) * distance, Math.Sin(h) * distance);
    }

    public override string ToString() => $"N{N:F1} E{E:F1}";
}

public static class GeoMath
{
    public const double EarthRadius = 6378137.0;
    const double Deg = Math.PI / 180;

    public static GeoPoint Offset(GeoPoint p, double north, double east, double? alt = null) =>
        new(p.Lat + north / EarthRadius / Deg,
            p.Lon + east / (EarthRadius * Math.Cos(p.Lat * Deg)) / Deg,
            alt ?? p.Alt);

    public static Vec2 ToLocal(GeoPoint origin, GeoPoint p) =>
        new((p.Lat - origin.Lat) * Deg * EarthRadius,
            (p.Lon - origin.Lon) * Deg * EarthRadius * Math.Cos(origin.Lat * Deg));

    public static GeoPoint FromLocal(GeoPoint origin, Vec2 v, double alt) => Offset(origin, v.N, v.E, alt);

    public static double Distance(GeoPoint a, GeoPoint b) => ToLocal(a, b).Length;

    public static double NormalizeDeg(double d)
    {
        d %= 360;
        return d < 0 ? d + 360 : d;
    }

    /// <summary>Signed smallest difference b - a in degrees (-180..180].</summary>
    public static double AngleDiff(double a, double b)
    {
        var d = NormalizeDeg(b - a);
        return d > 180 ? d - 360 : d;
    }
}
