using SkyHive.Core.Geo;
using SkyHive.Core.Vision;

namespace SkyHive.Core.Swarm;

public sealed record CameraFrame(RgbImage Image, IReadOnlyList<Blob> Blobs, double Gsd, double Altitude, double Heading, string Drone);

/// <summary>Runs every airborne drone's camera through the detection pipeline and feeds the shared tracker.</summary>
public sealed class VisionService(SyntheticWorld world, TargetTracker tracker, Func<IReadOnlyList<DroneAgent>> agents)
{
    int _rr;
    int _frame;
    public CameraModel Camera { get; } = new();
    public long FramesProcessed { get; private set; }
    public double LastProcessMs { get; private set; }

    public CameraFrame Capture(DroneAgent a, VisionMode mode)
    {
        var alt = Math.Max(a.Alt, 2);
        var raw = CameraRenderer.Render(world, Camera, a.Local, alt, a.State.Heading, world.SnapshotTargets(), ++_frame);
        var blobs = mode == VisionMode.Detection || mode == VisionMode.Thermal ? ImageOps.DetectRedTargets(raw) : (IReadOnlyList<Blob>)Array.Empty<Blob>();
        return new CameraFrame(ImageOps.Apply(raw, mode), blobs, Camera.GroundSampleDistance(alt), a.Alt, a.State.Heading, a.Name);
    }

    public async Task RunAsync(CancellationToken ct)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(200));
        while (await timer.WaitForNextTickAsync(ct).ConfigureAwait(false))
        {
            var list = agents().Where(a => a.State.Airborne && a.Alt > 3).ToList();
            if (list.Count == 0) continue;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            for (int k = 0; k < Math.Min(2, list.Count); k++)
                ProcessOne(list[_rr++ % list.Count]);
            LastProcessMs = sw.Elapsed.TotalMilliseconds;
        }
    }

    public int ProcessOne(DroneAgent a)
    {
        var alt = a.Alt;
        var pos = a.Local;
        var heading = a.State.Heading;
        var raw = CameraRenderer.Render(world, Camera, pos, alt, heading, world.SnapshotTargets(), ++_frame);
        var gsd = Camera.GroundSampleDistance(alt);
        int reported = 0;
        foreach (var b in ImageOps.DetectRedTargets(raw))
        {
            var areaM2 = b.Area * gsd * gsd;
            if (areaM2 < 0.4 || areaM2 > 40) continue;
            // ignore blobs cut by the frame edge (partial objects give biased centroids)
            if (b.MinX <= 0 || b.MinY <= 0 || b.MaxX >= Camera.Width - 1 || b.MaxY >= Camera.Height - 1) continue;
            var ground = Camera.PixelToGround(b.CenterX, b.CenterY, pos, alt, heading);
            tracker.Report(ground, b.Confidence, Math.Sqrt(areaM2), a.Name);
            reported++;
        }
        FramesProcessed++;
        return reported;
    }
}
