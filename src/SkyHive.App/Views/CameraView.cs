using SkiaSharp;
using SkiaSharp.Views.Maui;
using SkiaSharp.Views.Maui.Controls;
using SkyHive.Core.Swarm;
using SkyHive.Core.Vision;

namespace SkyHive.App.Views;

/// <summary>Gimbal camera feed with HUD and detection overlay.</summary>
public sealed class CameraView : SKCanvasView
{
    SKBitmap? _bitmap;
    CameraFrame? _frame;
    readonly DateTime _t0 = DateTime.UtcNow;

    public VisionMode Mode { get; set; } = VisionMode.Detection;
    public int ColorIndex { get; set; }

    public void SetFrame(CameraFrame? frame)
    {
        var old = _bitmap;
        _frame = frame;
        _bitmap = frame is null ? null : SkiaImage.ToBitmap(frame.Image);
        old?.Dispose();
        InvalidateSurface();
    }

    protected override void OnPaintSurface(SKPaintSurfaceEventArgs e)
    {
        var c = e.Surface.Canvas;
        float scale = (float)(e.Info.Width / Math.Max(1, Width));
        c.Clear(SKColor.Parse("#05080F"));
        c.Save();
        c.Scale(scale);
        float w = e.Info.Width / scale, h = e.Info.Height / scale;
        var accent = Palette.Accent;
        using var font = Palette.Font(11, true);
        using var small = Palette.Font(9.5f);
        using var text = new SKPaint { Color = Palette.Text, IsAntialias = true };

        var frame = _frame;
        if (_bitmap is null || frame is null)
        {
            using var mp = new SKPaint { Color = Palette.Muted, IsAntialias = true };
            c.DrawText("KAMERA BEKLENİYOR", w / 2, h / 2 - 4, SKTextAlign.Center, font, mp);
            c.DrawText("Drone seçin · kalkış sonrası görüntü akar", w / 2, h / 2 + 12, SKTextAlign.Center, small, mp);
            c.Restore();
            return;
        }

        // image (aspect fill)
        float iw = _bitmap.Width, ih = _bitmap.Height;
        float s = Math.Max(w / iw, h / ih);
        float dx = (w - iw * s) / 2, dy = (h - ih * s) / 2;
        using (var img = SKImage.FromBitmap(_bitmap))
        using (var ip = new SKPaint())
            c.DrawImage(img, new SKRect(dx, dy, dx + iw * s, dy + ih * s), new SKSamplingOptions(SKFilterMode.Linear), ip);

        // scanlines
        using (var sl = new SKPaint { Color = SKColors.Black.WithAlpha(28) })
            for (float y = 0; y < h; y += 3) c.DrawLine(0, y, w, y, sl);

        // detections
        if (frame.Blobs.Count > 0)
        {
            using var bp = new SKPaint { Color = Palette.Danger, Style = SKPaintStyle.Stroke, StrokeWidth = 1.6f, IsAntialias = true };
            using var lbg = new SKPaint { Color = Palette.Danger.WithAlpha(220), IsAntialias = true };
            foreach (var b in frame.Blobs)
            {
                var r = new SKRect(dx + (b.MinX - 2) * s, dy + (b.MinY - 2) * s, dx + (b.MaxX + 3) * s, dy + (b.MaxY + 3) * s);
                float k = Math.Min(10, r.Width / 3);
                // corner brackets
                c.DrawLine(r.Left, r.Top, r.Left + k, r.Top, bp); c.DrawLine(r.Left, r.Top, r.Left, r.Top + k, bp);
                c.DrawLine(r.Right, r.Top, r.Right - k, r.Top, bp); c.DrawLine(r.Right, r.Top, r.Right, r.Top + k, bp);
                c.DrawLine(r.Left, r.Bottom, r.Left + k, r.Bottom, bp); c.DrawLine(r.Left, r.Bottom, r.Left, r.Bottom - k, bp);
                c.DrawLine(r.Right, r.Bottom, r.Right - k, r.Bottom, bp); c.DrawLine(r.Right, r.Bottom, r.Right, r.Bottom - k, bp);
                var size = Math.Sqrt(b.Area) * frame.Gsd;
                var label = $"HEDEF {b.Confidence * 100:F0}% · {size:F1}m";
                var lw = small.MeasureText(label) + 8;
                c.DrawRoundRect(new SKRect(r.Left, r.Top - 15, r.Left + lw, r.Top - 2), 3, 3, lbg);
                c.DrawText(label, r.Left + 4, r.Top - 5, SKTextAlign.Left, small, text);
            }
        }

        // crosshair
        using (var cp = new SKPaint { Color = accent.WithAlpha(200), StrokeWidth = 1.2f, IsAntialias = true })
        {
            float cx = w / 2, cy = h / 2;
            c.DrawLine(cx - 18, cy, cx - 6, cy, cp); c.DrawLine(cx + 6, cy, cx + 18, cy, cp);
            c.DrawLine(cx, cy - 18, cx, cy - 6, cp); c.DrawLine(cx, cy + 6, cx, cy + 18, cp);
            cp.Style = SKPaintStyle.Stroke;
            c.DrawCircle(cx, cy, 2, cp);
            // frame corners
            float m = 10, L = 22;
            c.DrawLine(m, m, m + L, m, cp); c.DrawLine(m, m, m, m + L, cp);
            c.DrawLine(w - m, m, w - m - L, m, cp); c.DrawLine(w - m, m, w - m, m + L, cp);
            c.DrawLine(m, h - m, m + L, h - m, cp); c.DrawLine(m, h - m, m, h - m - L, cp);
            c.DrawLine(w - m, h - m, w - m - L, h - m, cp); c.DrawLine(w - m, h - m, w - m, h - m - L, cp);
        }

        // top bar
        using (var bar = new SKPaint { Color = SKColors.Black.WithAlpha(140) })
        {
            c.DrawRect(0, 0, w, 24, bar);
            c.DrawRect(0, h - 22, w, 22, bar);
        }
        bool blink = ((DateTime.UtcNow - _t0).TotalSeconds % 1.2) < 0.7;
        using (var rec = new SKPaint { Color = blink ? Palette.Danger : Palette.Danger.WithAlpha(60), IsAntialias = true })
            c.DrawCircle(w - 16, 12, 4, rec);
        using (var dp = new SKPaint { Color = Palette.Drone(ColorIndex), IsAntialias = true })
            c.DrawText(frame.Drone, 22, 16, SKTextAlign.Left, font, dp);
        c.DrawText($"ALT {frame.Altitude:F1} m   HDG {frame.Heading:000}°   GSD {frame.Gsd * 100:F0} cm/px", 52, 16, SKTextAlign.Left, small, text);
        c.DrawText("REC", w - 24, 16, SKTextAlign.Right, small, text);

        // heading tape
        using (var tp = new SKPaint { Color = Palette.Text.WithAlpha(190), StrokeWidth = 1, IsAntialias = true })
        {
            float cx = w / 2, y = h - 11;
            for (int d = -40; d <= 40; d += 10)
            {
                var hd = (frame.Heading + d + 360) % 360;
                float x = cx + d * 2.2f - (float)(frame.Heading % 10) * 2.2f;
                var mark = (int)Math.Round((frame.Heading + d - frame.Heading % 10 + 360) % 360);
                c.DrawLine(x, h - 22, x, h - 17, tp);
                if (mark % 30 == 0)
                {
                    var lbl = mark switch { 0 => "K", 90 => "D", 180 => "G", 270 => "B", _ => (mark / 10).ToString() };
                    c.DrawText(lbl, x, y + 4, SKTextAlign.Center, small, tp);
                }
                _ = hd;
            }
            using var tri = new SKPath();
            tri.MoveTo(cx, h - 22); tri.LineTo(cx - 4, h - 27); tri.LineTo(cx + 4, h - 27); tri.Close();
            using var trp = new SKPaint { Color = accent, IsAntialias = true };
            c.DrawPath(tri, trp);
        }
        var modeName = Mode switch
        {
            VisionMode.Rgb => "EO / RGB",
            VisionMode.Edges => "SOBEL KENAR",
            VisionMode.Thermal => "TERMAL (sim)",
            VisionMode.Vegetation => "BİTKİ İNDEKSİ",
            _ => $"TESPİT · {frame.Blobs.Count} nesne",
        };
        using (var mp = new SKPaint { Color = accent, IsAntialias = true })
        using (var mbg = new SKPaint { Color = SKColors.Black.WithAlpha(150), IsAntialias = true })
        {
            var mw = small.MeasureText(modeName) + 12;
            c.DrawRoundRect(new SKRect(w - mw - 12, 30, w - 12, 46), 4, 4, mbg);
            c.DrawText(modeName, w - mw - 6, 41.5f, SKTextAlign.Left, small, mp);
        }
        c.Restore();
    }
}
