using SkyHive.Core.Geo;

namespace SkyHive.Core.Vision;

/// <summary>Nadir (straight-down) gimbal camera model.</summary>
public sealed class CameraModel
{
    public int Width { get; init; } = 192;
    public int Height { get; init; } = 144;
    public double HorizontalFovDeg { get; init; } = 78;

    /// <summary>Metres per pixel at an altitude.</summary>
    public double GroundSampleDistance(double altitude) =>
        2 * Math.Max(altitude, 2) * Math.Tan(HorizontalFovDeg / 2 * Math.PI / 180) / Width;

    public double FootprintWidth(double altitude) => GroundSampleDistance(altitude) * Width;

    public Vec2 PixelToGround(double u, double v, Vec2 dronePos, double altitude, double headingDeg)
    {
        var gsd = GroundSampleDistance(altitude);
        return dronePos + Vec2.FromBody((Height / 2.0 - v) * gsd, (u - Width / 2.0) * gsd, headingDeg);
    }
}

public static class CameraRenderer
{
    /// <summary>Renders what a nadir camera at <paramref name="pos"/> sees in the synthetic world.</summary>
    public static RgbImage Render(SyntheticWorld world, CameraModel cam, Vec2 pos, double altitude, double headingDeg,
        IReadOnlyList<WorldTarget> targets, int noiseSeed = 0)
    {
        var img = new RgbImage(cam.Width, cam.Height);
        var gsd = cam.GroundSampleDistance(altitude);
        var h = headingDeg * Math.PI / 180;
        double ch = Math.Cos(h), sh = Math.Sin(h);
        var rng = new Random(noiseSeed);
        double cx = cam.Width / 2.0, cy = cam.Height / 2.0, maxR = Math.Sqrt(cx * cx + cy * cy);
        // only test targets that can be inside the footprint
        var radius = Math.Sqrt(cam.Width * cam.Width + cam.Height * cam.Height) * gsd / 2 + 5;
        var visible = targets.Where(t => t.Position.DistanceTo(pos) < radius).ToList();

        for (int y = 0; y < cam.Height; y++)
        {
            double fwd = (cy - y - 0.5) * gsd;
            for (int x = 0; x < cam.Width; x++)
            {
                double right = (x + 0.5 - cx) * gsd;
                var p = new Vec2(pos.N + fwd * ch - right * sh, pos.E + fwd * sh + right * ch);
                var c = (visible.Count > 0 ? SyntheticWorld.SampleTargets(p, visible) : null) ?? world.SampleTerrain(p, gsd < 0.6 ? 1 : 0);
                // vignette + sensor noise
                double r = Math.Sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy)) / maxR;
                double vig = 1 - 0.28 * r * r;
                double noise = (rng.NextDouble() - 0.5) * 8;
                img.Set(x, y,
                    (byte)Math.Clamp(c.R * vig + noise, 0, 255),
                    (byte)Math.Clamp(c.G * vig + noise, 0, 255),
                    (byte)Math.Clamp(c.B * vig + noise, 0, 255));
            }
        }
        return img;
    }

    /// <summary>Top-down overview map of the world terrain (1 px = <paramref name="metresPerPixel"/>), north up.</summary>
    public static RgbImage RenderMap(SyntheticWorld world, int size, double metresPerPixel)
    {
        var img = new RgbImage(size, size);
        double half = size / 2.0;
        Parallel.For(0, size, y =>
        {
            for (int x = 0; x < size; x++)
            {
                var p = new Vec2((half - y - 0.5) * metresPerPixel, (x + 0.5 - half) * metresPerPixel);
                var c = world.SampleTerrain(p, 0);
                img.Set(x, y, c.R, c.G, c.B);
            }
        });
        return img;
    }
}
