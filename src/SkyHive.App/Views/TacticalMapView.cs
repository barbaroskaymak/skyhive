using SkiaSharp;
using SkiaSharp.Views.Maui;
using SkiaSharp.Views.Maui.Controls;
using SkyHive.Core;
using SkyHive.Core.Geo;
using SkyHive.Core.Swarm;
using SkyHive.Core.Vision;

namespace SkyHive.App.Views;

/// <summary>Skia-rendered tactical map: terrain, trails, tasks, camera footprints, detections and drones.</summary>
public sealed class TacticalMapView : SKCanvasView
{
    SKBitmap? _terrain;
    const double TerrainMpp = 1.0;
    const int TerrainSize = 1200;
    double _pinchStartZoom;
    Vec2 _panStartCenter;
    readonly DateTime _t0 = DateTime.UtcNow;

    public TacticalMapView()
    {
        EnableTouchEvents = false;
        var pan = new PanGestureRecognizer();
        pan.PanUpdated += OnPan;
        var pinch = new PinchGestureRecognizer();
        pinch.PinchUpdated += OnPinch;
        var tap = new TapGestureRecognizer();
        tap.Tapped += OnTap;
        GestureRecognizers.Add(pan);
        GestureRecognizers.Add(pinch);
        GestureRecognizers.Add(tap);
    }

    public SwarmSession? Session { get; set; }
    public int? SelectedId { get; set; }
    public bool FollowSelected { get; set; }
    public bool TapToGo { get; set; }
    public double Zoom { get; set; } = 2.2; // pixels per metre
    public Vec2 Center { get; set; } = new(20, 0);
    public event Action<Vec2>? GroundTapped;
    public event Action<int>? DroneTapped;

    public void LoadTerrain(SyntheticWorld world)
    {
        Task.Run(() =>
        {
            var img = CameraRenderer.RenderMap(world, TerrainSize, TerrainMpp);
            var bmp = SkiaImage.ToBitmap(img);
            MainThread.BeginInvokeOnMainThread(() => { _terrain = bmp; InvalidateSurface(); });
        });
    }

    public void ZoomBy(double factor) => Zoom = Math.Clamp(Zoom * factor, 0.35, 14);

    public void CenterOnSwarm()
    {
        if (Session is null) return;
        Center = Session.Controller.Centroid();
    }

    // ------------------------------------------------------------ gestures

    void OnPan(object? s, PanUpdatedEventArgs e)
    {
        if (e.StatusType == GestureStatus.Started) { _panStartCenter = Center; FollowSelected = false; }
        if (e.StatusType == GestureStatus.Running)
            Center = new Vec2(_panStartCenter.N + e.TotalY / Zoom, _panStartCenter.E - e.TotalX / Zoom);
    }

    void OnPinch(object? s, PinchGestureUpdatedEventArgs e)
    {
        if (e.Status == GestureStatus.Started) _pinchStartZoom = Zoom;
        if (e.Status == GestureStatus.Running) Zoom = Math.Clamp(Zoom * e.Scale, 0.35, 14);
    }

    void OnTap(object? s, TappedEventArgs e)
    {
        var pt = e.GetPosition(this);
        if (pt is null || Session is null) return;
        var ground = ScreenToWorld((float)pt.Value.X, (float)pt.Value.Y, (float)Width, (float)Height);
        var nearest = Session.Agents.OrderBy(a => a.Local.DistanceTo(ground)).FirstOrDefault();
        if (!TapToGo && nearest is not null && nearest.Local.DistanceTo(ground) * Zoom < 28) { DroneTapped?.Invoke(nearest.Id); return; }
        if (TapToGo) GroundTapped?.Invoke(ground);
    }

    Vec2 ScreenToWorld(float x, float y, float w, float h) =>
        new(Center.N - (y - h / 2) / Zoom, Center.E + (x - w / 2) / Zoom);

    // ------------------------------------------------------------- drawing

    protected override void OnPaintSurface(SKPaintSurfaceEventArgs e)
    {
        var c = e.Surface.Canvas;
        var info = e.Info;
        float scale = (float)(info.Width / Math.Max(1, Width)); // device pixels per DIP
        c.Clear(Palette.Bg);
        c.Save();
        c.Scale(scale);
        float w = info.Width / scale, h = info.Height / scale;

        var session = Session;
        if (session is not null && FollowSelected && SelectedId is { } sid && session.Controller.Find(sid) is { } follow)
            Center = Center * 0.85 + follow.Local * 0.15;

        SKPoint P(Vec2 v) => new((float)(w / 2 + (v.E - Center.E) * Zoom), (float)(h / 2 - (v.N - Center.N) * Zoom));
        float time = (float)(DateTime.UtcNow - _t0).TotalSeconds;

        // terrain
        if (_terrain is not null)
        {
            var tl = P(new Vec2(TerrainSize / 2.0 * TerrainMpp, -TerrainSize / 2.0 * TerrainMpp));
            var br = P(new Vec2(-TerrainSize / 2.0 * TerrainMpp, TerrainSize / 2.0 * TerrainMpp));
            using var sampling = new SKPaint { IsAntialias = true };
            c.DrawImage(SKImage.FromBitmap(_terrain), new SKRect(tl.X, tl.Y, br.X, br.Y), new SKSamplingOptions(SKFilterMode.Linear), sampling);
            using var dim = new SKPaint { Color = Palette.Bg.WithAlpha(105) };
            c.DrawRect(0, 0, w, h, dim);
        }

        DrawGrid(c, w, h, P);
        if (session is null)
        {
            using var f = Palette.Font(16, true);
            using var tp = new SKPaint { Color = Palette.Muted, IsAntialias = true };
            c.DrawText("Bağlantı yok — üstten BAĞLAN'a basın", w / 2, h / 2, SKTextAlign.Center, f, tp);
            c.Restore();
            return;
        }

        // home
        using (var hp = new SKPaint { Color = Palette.Text.WithAlpha(200), IsAntialias = true, Style = SKPaintStyle.Stroke, StrokeWidth = 1.5f })
        using (var hf = Palette.Font(10, true))
        using (var ht = new SKPaint { Color = Palette.Text, IsAntialias = true })
        {
            var hpos = P(Vec2.Zero);
            c.DrawCircle(hpos, 11, hp);
            c.DrawText("H", hpos.X, hpos.Y + 3.5f, SKTextAlign.Center, hf, ht);
        }

        var agents = session.Agents;
        // task geometry
        foreach (var a in agents)
        {
            var col = Palette.Drone(a.Index);
            switch (a.Task)
            {
                case WaypointTask wt:
                {
                    var path = wt.PlannedPath;
                    if (path.Count == 0) break;
                    using var pp = new SKPaint { Color = col.WithAlpha(150), Style = SKPaintStyle.Stroke, StrokeWidth = 1.6f, IsAntialias = true, PathEffect = SKPathEffect.CreateDash(new[] { 6f, 5f }, -time * 12) };
                    using var path2 = new SKPath();
                    path2.MoveTo(P(a.Local));
                    foreach (var wp in path) path2.LineTo(P(wp));
                    c.DrawPath(path2, pp);
                    using var wpp = new SKPaint { Color = col.WithAlpha(200), IsAntialias = true };
                    foreach (var wp in path) c.DrawCircle(P(wp), 2.2f, wpp);
                    break;
                }
                case OrbitTask ot when ot.Center is { } oc:
                {
                    using var op = new SKPaint { Color = col.WithAlpha(110), Style = SKPaintStyle.Stroke, StrokeWidth = 1.2f, IsAntialias = true, PathEffect = SKPathEffect.CreateDash(new[] { 3f, 4f }, 0) };
                    c.DrawCircle(P(oc), (float)(ot.Radius * Zoom), op);
                    break;
                }
                case FormationTask ft when session.Controller.Find(ft.LeaderId) is { } leader:
                {
                    using var fp = new SKPaint { Color = col.WithAlpha(90), Style = SKPaintStyle.Stroke, StrokeWidth = 1f, IsAntialias = true };
                    c.DrawLine(P(leader.Local), P(a.Local), fp);
                    break;
                }
            }
            if (a.LastTarget is { } lt && a.Task is not null)
            {
                using var tp = new SKPaint { Color = col.WithAlpha(170), Style = SKPaintStyle.Stroke, StrokeWidth = 1.2f, IsAntialias = true };
                var q = P(lt.Position);
                c.DrawLine(q.X - 4, q.Y - 4, q.X + 4, q.Y + 4, tp);
                c.DrawLine(q.X - 4, q.Y + 4, q.X + 4, q.Y - 4, tp);
            }
        }

        // trails
        foreach (var a in agents)
        {
            var trail = a.Trail();
            if (trail.Count < 2) continue;
            var col = Palette.Drone(a.Index);
            using var tp = new SKPaint { Style = SKPaintStyle.Stroke, StrokeWidth = 2f, IsAntialias = true, StrokeCap = SKStrokeCap.Round };
            for (int i = 1; i < trail.Count; i++)
            {
                tp.Color = col.WithAlpha((byte)(20 + 150 * i / trail.Count));
                c.DrawLine(P(trail[i - 1]), P(trail[i]), tp);
            }
        }

        // camera footprints
        var cam = session.Vision.Camera;
        foreach (var a in agents.Where(a => a.State.Airborne))
        {
            bool sel = a.Id == SelectedId;
            var fw = cam.FootprintWidth(a.Alt) / 2;
            var fh = fw * cam.Height / cam.Width;
            using var path = new SKPath();
            path.MoveTo(P(a.Local + Vec2.FromBody(fh, -fw, a.State.Heading)));
            path.LineTo(P(a.Local + Vec2.FromBody(fh, fw, a.State.Heading)));
            path.LineTo(P(a.Local + Vec2.FromBody(-fh, fw, a.State.Heading)));
            path.LineTo(P(a.Local + Vec2.FromBody(-fh, -fw, a.State.Heading)));
            path.Close();
            using var fill = new SKPaint { Color = Palette.Drone(a.Index).WithAlpha(sel ? (byte)40 : (byte)16), IsAntialias = true };
            using var stroke = new SKPaint { Color = Palette.Drone(a.Index).WithAlpha(sel ? (byte)200 : (byte)70), Style = SKPaintStyle.Stroke, StrokeWidth = 1, IsAntialias = true };
            c.DrawPath(path, fill);
            c.DrawPath(path, stroke);
        }

        // detections
        using (var tf = Palette.Font(10.5f, true))
        foreach (var t in session.Tracker.Snapshot())
        {
            var p = P(t.Position);
            var col = t.Confirmed ? Palette.Danger : Palette.Warn;
            float pulse = (time * 1.4f + t.Id * 0.37f) % 1f;
            if (t.Confirmed)
            {
                using var ring = new SKPaint { Color = col.WithAlpha((byte)(170 * (1 - pulse))), Style = SKPaintStyle.Stroke, StrokeWidth = 1.5f, IsAntialias = true };
                c.DrawCircle(p, 8 + pulse * 18, ring);
            }
            using var dp = new SKPath();
            dp.MoveTo(p.X, p.Y - 7); dp.LineTo(p.X + 7, p.Y); dp.LineTo(p.X, p.Y + 7); dp.LineTo(p.X - 7, p.Y); dp.Close();
            using var df = new SKPaint { Color = col.WithAlpha(220), IsAntialias = true };
            using var ds = new SKPaint { Color = SKColors.White.WithAlpha(220), Style = SKPaintStyle.Stroke, StrokeWidth = 1.2f, IsAntialias = true };
            c.DrawPath(dp, df);
            c.DrawPath(dp, ds);
            if (t.Velocity.Length > 0.5)
            {
                using var vp = new SKPaint { Color = col, StrokeWidth = 1.5f, IsAntialias = true };
                c.DrawLine(p, P(t.Position + t.Velocity * 4), vp);
            }
            using var lp = new SKPaint { Color = SKColors.White, IsAntialias = true };
            c.DrawText(t.Label, p.X + 10, p.Y - 8, SKTextAlign.Left, tf, lp);
        }

        // drones
        using var nameFont = Palette.Font(11, true);
        using var altFont = Palette.Font(9.5f);
        foreach (var a in agents.Where(a => a.State.HasPosition))
        {
            var p = P(a.Local);
            var col = Palette.Drone(a.Index);
            bool sel = a.Id == SelectedId;
            if (sel)
            {
                using var sp = new SKPaint { Color = col, Style = SKPaintStyle.Stroke, StrokeWidth = 1.6f, IsAntialias = true, PathEffect = SKPathEffect.CreateDash(new[] { 5f, 4f }, time * 20) };
                c.DrawCircle(p, 19, sp);
            }
            using var glow = new SKPaint { Color = col.WithAlpha(a.State.Airborne ? (byte)70 : (byte)30), IsAntialias = true, MaskFilter = SKMaskFilter.CreateBlur(SKBlurStyle.Normal, 6) };
            c.DrawCircle(p, 12, glow);

            c.Save();
            c.Translate(p);
            c.RotateDegrees((float)a.State.Heading);
            using var body = new SKPath();
            body.MoveTo(0, -12); body.LineTo(8, 9); body.LineTo(0, 4.5f); body.LineTo(-8, 9); body.Close();
            using var bf = new SKPaint { Color = a.State.Connected ? col : Palette.Muted, IsAntialias = true };
            using var bs = new SKPaint { Color = SKColors.White.WithAlpha(230), Style = SKPaintStyle.Stroke, StrokeWidth = 1.3f, IsAntialias = true, StrokeJoin = SKStrokeJoin.Round };
            c.DrawPath(body, bf);
            c.DrawPath(body, bs);
            c.Restore();

            var label = $"{a.Name}";
            var sub = a.State.Airborne ? $"{a.Alt:F0} m · {a.State.GroundSpeed:F1} m/s" : a.State.Connected ? "yerde" : "bağlantı yok";
            var lw = Math.Max(nameFont.MeasureText(label), altFont.MeasureText(sub)) + 12;
            var rect = new SKRect(p.X + 14, p.Y - 26, p.X + 14 + lw, p.Y - 2);
            using var lb = new SKPaint { Color = Palette.Panel.WithAlpha(215), IsAntialias = true };
            using var lbs = new SKPaint { Color = col.WithAlpha(160), Style = SKPaintStyle.Stroke, StrokeWidth = 1, IsAntialias = true };
            c.DrawRoundRect(rect, 5, 5, lb);
            c.DrawRoundRect(rect, 5, 5, lbs);
            using var tp = new SKPaint { Color = col, IsAntialias = true };
            using var tp2 = new SKPaint { Color = Palette.Text.WithAlpha(210), IsAntialias = true };
            c.DrawText(label, rect.Left + 6, rect.Top + 11, SKTextAlign.Left, nameFont, tp);
            c.DrawText(sub, rect.Left + 6, rect.Top + 21.5f, SKTextAlign.Left, altFont, tp2);
        }

        DrawHud(c, w, h);
        c.Restore();
    }

    void DrawGrid(SKCanvas c, float w, float h, Func<Vec2, SKPoint> P)
    {
        double step = Zoom > 4 ? 25 : Zoom > 1.2 ? 50 : 100;
        var tl = ScreenToWorld(0, 0, w, h);
        var br = ScreenToWorld(w, h, w, h);
        using var minor = new SKPaint { Color = Palette.Accent.WithAlpha(18), StrokeWidth = 1 };
        using var major = new SKPaint { Color = Palette.Accent.WithAlpha(45), StrokeWidth = 1 };
        using var lf = Palette.Font(9);
        using var lp = new SKPaint { Color = Palette.Accent.WithAlpha(110), IsAntialias = true };
        for (double e = Math.Floor(tl.E / step) * step; e <= br.E; e += step)
        {
            var x = P(new Vec2(0, e)).X;
            bool isMajor = Math.Abs(e % (step * 5)) < 0.01;
            c.DrawLine(x, 0, x, h, isMajor ? major : minor);
            if (isMajor) c.DrawText($"{e:F0}", x + 3, h - 6, SKTextAlign.Left, lf, lp);
        }
        for (double n = Math.Floor(br.N / step) * step; n <= tl.N; n += step)
        {
            var y = P(new Vec2(n, 0)).Y;
            bool isMajor = Math.Abs(n % (step * 5)) < 0.01;
            c.DrawLine(0, y, w, y, isMajor ? major : minor);
            if (isMajor) c.DrawText($"{n:F0}", 4, y - 3, SKTextAlign.Left, lf, lp);
        }
    }

    void DrawHud(SKCanvas c, float w, float h)
    {
        // compass
        var cp = new SKPoint(w - 46, 46);
        using var ring = new SKPaint { Color = Palette.Panel.WithAlpha(200), IsAntialias = true };
        using var ringS = new SKPaint { Color = Palette.Accent.WithAlpha(120), Style = SKPaintStyle.Stroke, StrokeWidth = 1.2f, IsAntialias = true };
        c.DrawCircle(cp, 26, ring);
        c.DrawCircle(cp, 26, ringS);
        using var n = new SKPath();
        n.MoveTo(cp.X, cp.Y - 20); n.LineTo(cp.X + 6, cp.Y); n.LineTo(cp.X - 6, cp.Y); n.Close();
        using var np = new SKPaint { Color = Palette.Danger, IsAntialias = true };
        c.DrawPath(n, np);
        using var s = new SKPath();
        s.MoveTo(cp.X, cp.Y + 20); s.LineTo(cp.X + 6, cp.Y); s.LineTo(cp.X - 6, cp.Y); s.Close();
        using var spnt = new SKPaint { Color = Palette.Text.WithAlpha(160), IsAntialias = true };
        c.DrawPath(s, spnt);
        using var f = Palette.Font(9, true);
        using var tp = new SKPaint { Color = Palette.Text, IsAntialias = true };
        c.DrawText("K", cp.X, cp.Y - 29, SKTextAlign.Center, f, tp);

        // scale bar
        double[] nice = { 5, 10, 20, 25, 50, 100, 200, 250, 500 };
        var metres = nice.FirstOrDefault(m => m * Zoom > 70, 500);
        float len = (float)(metres * Zoom);
        var y = h - 26;
        using var sb = new SKPaint { Color = Palette.Text, StrokeWidth = 2, IsAntialias = true };
        using var bg = new SKPaint { Color = Palette.Panel.WithAlpha(190), IsAntialias = true };
        c.DrawRoundRect(new SKRect(36, y - 18, 36 + len + 60, y + 9), 6, 6, bg);
        c.DrawLine(44, y, 44 + len, y, sb);
        c.DrawLine(44, y - 4, 44, y + 4, sb);
        c.DrawLine(44 + len, y - 4, 44 + len, y + 4, sb);
        c.DrawText($"{metres:F0} m", 50 + len, y + 3.5f, SKTextAlign.Left, f, tp);
        c.DrawText(TapToGo ? "DOKUN-GİT AKTİF" : $"x{Zoom:F1}", 44, y - 7, SKTextAlign.Left, f, new SKPaint { Color = TapToGo ? Palette.Warn : Palette.Muted, IsAntialias = true });
    }
}
