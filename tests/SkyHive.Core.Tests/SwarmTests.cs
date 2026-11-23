using SkyHive.Core.Commands;
using SkyHive.Core.Geo;
using SkyHive.Core.Links;
using SkyHive.Core.Model;
using SkyHive.Core.Swarm;
using SkyHive.Core.Vision;

namespace SkyHive.Core.Tests;

public class GeoAndPlanningTests
{
    [Fact]
    public void Local_roundtrip_is_centimetre_accurate()
    {
        var o = new GeoPoint(40.9925, 29.124, 0);
        var p = GeoMath.FromLocal(o, new Vec2(250, -130), 30);
        var back = GeoMath.ToLocal(o, p);
        Assert.InRange(back.N, 249.99, 250.01);
        Assert.InRange(back.E, -130.01, -129.99);
    }

    [Theory]
    [InlineData(FormationType.Line)]
    [InlineData(FormationType.Column)]
    [InlineData(FormationType.Wedge)]
    [InlineData(FormationType.Circle)]
    [InlineData(FormationType.Grid)]
    public void Formation_slots_keep_minimum_spacing(FormationType type)
    {
        var slots = Formations.Slots(type, 7, 10);
        Assert.Equal(7, slots.Count);
        Assert.Equal((0.0, 0.0), slots[0]);
        for (int i = 0; i < slots.Count; i++)
            for (int j = i + 1; j < slots.Count; j++)
            {
                var d = Math.Sqrt(Math.Pow(slots[i].Forward - slots[j].Forward, 2) + Math.Pow(slots[i].Right - slots[j].Right, 2));
                Assert.True(d >= 9.99, $"{type}: slots {i},{j} only {d:F2} m apart");
            }
    }

    [Fact]
    public void Lawnmower_covers_area_in_disjoint_strips()
    {
        var paths = PathPlanner.LawnmowerSplit(new Vec2(0, 0), 300, 200, 4, 20);
        Assert.Equal(4, paths.Count);
        for (int d = 0; d < 4; d++)
        {
            double left = -150 + d * 75, right = left + 75;
            Assert.All(paths[d], p => Assert.InRange(p.E, left, right));
            Assert.All(paths[d], p => Assert.InRange(p.N, -100, 100));
        }
        Assert.Equal(16, paths.Sum(p => p.Count) / 2); // 4 lanes per strip
    }
}

public class SimulatorTests
{
    static readonly GeoPoint Home = new(40.9925, 29.124, 0);

    [Fact]
    public async Task Takeoff_goto_land_cycle()
    {
        var sim = new SimDroneLink(1, Home);
        Assert.False(await sim.ArmAsync(true)); // no GPS fix yet
        for (int i = 0; i < 40; i++) sim.Step(0.05);
        await sim.SetModeAsync(FlightMode.Guided);
        Assert.True(await sim.TakeoffAsync(20));
        for (int i = 0; i < 400; i++) sim.Step(0.05);
        Assert.InRange(sim.State.Position.Alt, 19.5, 20.5);

        await sim.GotoAsync(GeoMath.Offset(Home, 60, 30, 25));
        for (int i = 0; i < 600; i++) sim.Step(0.05);
        var local = GeoMath.ToLocal(Home, sim.State.Position);
        Assert.True(local.DistanceTo(new Vec2(60, 30)) < 2.5, $"at {local}");

        await sim.LandAsync();
        for (int i = 0; i < 800; i++) sim.Step(0.05);
        Assert.False(sim.State.Armed);
        Assert.Equal(0, sim.State.Position.Alt, 2);
    }

    [Fact]
    public async Task Rtl_returns_home_and_disarms()
    {
        var sim = new SimDroneLink(2, Home);
        for (int i = 0; i < 40; i++) sim.Step(0.05);
        await sim.TakeoffAsync(10);
        for (int i = 0; i < 200; i++) sim.Step(0.05);
        await sim.GotoAsync(GeoMath.Offset(Home, -80, 40, 12));
        for (int i = 0; i < 600; i++) sim.Step(0.05);
        await sim.ReturnToLaunchAsync();
        for (int i = 0; i < 2000 && sim.State.Armed; i++) sim.Step(0.05);
        Assert.False(sim.State.Armed);
        Assert.True(GeoMath.Distance(Home, sim.State.Position) < 2);
    }
}

public class VisionTests
{
    [Fact]
    public void Detects_and_georeferences_targets_in_synthetic_frame()
    {
        var world = new SyntheticWorld(1337);
        var cam = new CameraModel();
        var target = world.SnapshotTargets().First(t => t.Speed == 0 && t.Kind == TargetKind.Vehicle);
        var dronePos = target.Position + new Vec2(4, -3);
        var img = CameraRenderer.Render(world, cam, dronePos, 25, 30, world.SnapshotTargets());
        var blobs = ImageOps.DetectRedTargets(img);
        var hit = blobs.Select(b => cam.PixelToGround(b.CenterX, b.CenterY, dronePos, 25, 30))
            .OrderBy(p => p.DistanceTo(target.Position)).First();
        Assert.True(hit.DistanceTo(target.Position) < 1.0, $"error {hit.DistanceTo(target.Position):F2} m");
    }

    [Fact]
    public void Terrain_without_targets_produces_no_detections()
    {
        var world = new SyntheticWorld(1337);
        var cam = new CameraModel();
        int falsePositives = 0;
        for (int i = 0; i < 25; i++)
        {
            var pos = new Vec2(-500 + i * 40, 300 - i * 23);
            falsePositives += ImageOps.DetectRedTargets(CameraRenderer.Render(world, cam, pos, 30, i * 15, Array.Empty<WorldTarget>())).Count;
        }
        Assert.Equal(0, falsePositives);
    }

    [Fact]
    public void Tracker_fuses_repeated_detections()
    {
        var tr = new TargetTracker();
        var t0 = DateTime.UtcNow;
        for (int i = 0; i < 5; i++) tr.Report(new Vec2(100 + i * 0.3, 50), 0.8, 3, "D1", t0.AddSeconds(i));
        tr.Report(new Vec2(-200, 10), 0.8, 3, "D2", t0.AddSeconds(5));
        var list = tr.Snapshot();
        Assert.Equal(2, list.Count);
        Assert.True(list[0].Confirmed);
        Assert.False(list[1].Confirmed);
    }
}

public class CommandTests
{
    [Fact]
    public async Task Full_swarm_mission_via_text_commands()
    {
        await using var s = await SwarmSession.StartAsync(new SessionOptions { DroneCount = 4 });
        foreach (var a in s.Agents) ((SimDroneLink)a.Link).TimeScale = 6; // fast-forward
        await Task.Delay(600);

        var r = await s.Commands.ExecuteAsync("kalk hepsi 20");
        Assert.True(r.Ok, r.Message);
        await WaitFor(() => s.Agents.All(a => a.Alt > 19), 20);

        r = await s.Commands.ExecuteAsync("formasyon v 12");
        Assert.True(r.Ok, r.Message);
        Assert.All(s.Agents.Skip(1), a => Assert.IsType<FormationTask>(a.Task));

        r = await s.Commands.ExecuteAsync("tara 200x160");
        Assert.True(r.Ok, r.Message);
        Assert.All(s.Agents, a => Assert.IsType<WaypointTask>(a.Task));
        await WaitFor(() => s.Agents.Any(a => a.Local.Length > 40), 20);

        r = await s.Commands.ExecuteAsync("eve dön");
        Assert.True(r.Ok, r.Message);
        Assert.All(s.Agents, a => Assert.True(a.State.Mode == FlightMode.Rtl, r.Message + " | " + string.Join(" ", s.LogSnapshot().TakeLast(12).Select(l => l.Source + ":" + l.Message))));

        Assert.False((await s.Commands.ExecuteAsync("uç uç")).Ok);
    }

    static async Task WaitFor(Func<bool> cond, double seconds)
    {
        var end = DateTime.UtcNow.AddSeconds(seconds);
        while (!cond())
        {
            if (DateTime.UtcNow > end) throw new TimeoutException();
            await Task.Delay(100);
        }
    }
}
