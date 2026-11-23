namespace SkyHive.Core.Vision;

public enum VisionMode { Rgb, Edges, Thermal, Vegetation, Detection }

public sealed record Blob(int MinX, int MinY, int MaxX, int MaxY, double CenterX, double CenterY, int Area, double Confidence)
{
    public int W => MaxX - MinX + 1;
    public int H => MaxY - MinY + 1;
}

/// <summary>Classic image-processing primitives written from scratch (no OpenCV dependency, runs on every MAUI target).</summary>
public static class ImageOps
{
    public static GrayImage Grayscale(RgbImage img)
    {
        var g = new GrayImage(img.Width, img.Height);
        var p = img.Pixels;
        for (int i = 0, j = 0; j < g.Data.Length; i += 4, j++)
            g.Data[j] = (0.299f * p[i] + 0.587f * p[i + 1] + 0.114f * p[i + 2]) / 255f;
        return g;
    }

    public static GrayImage GaussianBlur3(GrayImage s)
    {
        var o = new GrayImage(s.Width, s.Height);
        for (int y = 0; y < s.Height; y++)
            for (int x = 0; x < s.Width; x++)
            {
                float sum = 0, w = 0;
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        int xx = x + dx, yy = y + dy;
                        if (xx < 0 || yy < 0 || xx >= s.Width || yy >= s.Height) continue;
                        float k = (dx == 0 ? 2 : 1) * (dy == 0 ? 2 : 1);
                        sum += s[xx, yy] * k; w += k;
                    }
                o[x, y] = sum / w;
            }
        return o;
    }

    /// <summary>Sobel gradient magnitude (0..~1).</summary>
    public static GrayImage Sobel(GrayImage s)
    {
        var o = new GrayImage(s.Width, s.Height);
        for (int y = 1; y < s.Height - 1; y++)
            for (int x = 1; x < s.Width - 1; x++)
            {
                float gx = -s[x - 1, y - 1] - 2 * s[x - 1, y] - s[x - 1, y + 1] + s[x + 1, y - 1] + 2 * s[x + 1, y] + s[x + 1, y + 1];
                float gy = -s[x - 1, y - 1] - 2 * s[x, y - 1] - s[x + 1, y - 1] + s[x - 1, y + 1] + 2 * s[x, y + 1] + s[x + 1, y + 1];
                o[x, y] = MathF.Min(1, MathF.Sqrt(gx * gx + gy * gy) / 2.5f);
            }
        return o;
    }

    public static (float H, float S, float V) ToHsv(byte r, byte g, byte b)
    {
        float rf = r / 255f, gf = g / 255f, bf = b / 255f;
        float max = MathF.Max(rf, MathF.Max(gf, bf)), min = MathF.Min(rf, MathF.Min(gf, bf)), d = max - min;
        float h = 0;
        if (d > 1e-6f)
        {
            if (max == rf) h = 60 * (((gf - bf) / d) % 6);
            else if (max == gf) h = 60 * ((bf - rf) / d + 2);
            else h = 60 * ((rf - gf) / d + 4);
        }
        if (h < 0) h += 360;
        return (h, max <= 0 ? 0 : d / max, max);
    }

    /// <summary>Binary mask of strongly saturated red pixels (target colour).</summary>
    public static bool[] RedMask(RgbImage img)
    {
        var mask = new bool[img.Width * img.Height];
        var p = img.Pixels;
        for (int i = 0, j = 0; j < mask.Length; i += 4, j++)
        {
            var (h, s, v) = ToHsv(p[i], p[i + 1], p[i + 2]);
            mask[j] = (h < 14 || h > 344) && s > 0.55f && v > 0.30f;
        }
        return mask;
    }

    public static bool[] Erode(bool[] m, int w, int h) => Morph(m, w, h, true);
    public static bool[] Dilate(bool[] m, int w, int h) => Morph(m, w, h, false);

    static bool[] Morph(bool[] m, int w, int h, bool erode)
    {
        var o = new bool[m.Length];
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
            {
                bool acc = erode;
                for (int dy = -1; dy <= 1 && acc == erode; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (Math.Abs(dx) + Math.Abs(dy) > 1) continue; // cross kernel
                        int xx = Math.Clamp(x + dx, 0, w - 1), yy = Math.Clamp(y + dy, 0, h - 1);
                        bool v = m[yy * w + xx];
                        if (erode && !v) { acc = false; break; }
                        if (!erode && v) { acc = true; break; }
                    }
                o[y * w + x] = acc;
            }
        return o;
    }

    /// <summary>4-connected component labelling → blobs above a minimum area.</summary>
    public static List<Blob> ConnectedComponents(bool[] mask, int w, int h, int minArea)
    {
        var labels = new int[mask.Length];
        var blobs = new List<Blob>();
        var stack = new Stack<int>();
        int label = 0;
        for (int start = 0; start < mask.Length; start++)
        {
            if (!mask[start] || labels[start] != 0) continue;
            label++;
            int minX = int.MaxValue, minY = int.MaxValue, maxX = -1, maxY = -1, area = 0;
            double sx = 0, sy = 0;
            stack.Push(start); labels[start] = label;
            while (stack.Count > 0)
            {
                int idx = stack.Pop();
                int x = idx % w, y = idx / w;
                area++; sx += x; sy += y;
                if (x < minX) minX = x; if (x > maxX) maxX = x;
                if (y < minY) minY = y; if (y > maxY) maxY = y;
                void Visit(int n) { if (mask[n] && labels[n] == 0) { labels[n] = label; stack.Push(n); } }
                if (x > 0) Visit(idx - 1);
                if (x < w - 1) Visit(idx + 1);
                if (y > 0) Visit(idx - w);
                if (y < h - 1) Visit(idx + w);
            }
            if (area < minArea) continue;
            double fill = area / (double)((maxX - minX + 1) * (maxY - minY + 1));
            double conf = Math.Clamp(0.45 + 0.35 * fill + Math.Min(area, 200) / 800.0, 0, 0.99);
            blobs.Add(new Blob(minX, minY, maxX, maxY, sx / area + 0.5, sy / area + 0.5, area, conf));
        }
        return blobs;
    }

    /// <summary>Full detection pipeline: HSV threshold → morphological opening → closing → connected components.</summary>
    public static List<Blob> DetectRedTargets(RgbImage img, int minArea = 6)
    {
        var m = RedMask(img);
        m = Dilate(Erode(m, img.Width, img.Height), img.Width, img.Height);  // opening removes speckle
        m = Erode(Dilate(m, img.Width, img.Height), img.Width, img.Height);  // closing fills windscreen holes
        return ConnectedComponents(m, img.Width, img.Height, minArea);
    }

    // ---------------- visualisation modes ----------------

    public static RgbImage EdgeView(RgbImage img)
    {
        var e = Sobel(GaussianBlur3(Grayscale(img)));
        var o = new RgbImage(img.Width, img.Height);
        for (int y = 0; y < img.Height; y++)
            for (int x = 0; x < img.Width; x++)
            {
                float v = MathF.Min(1, e[x, y] * 2.2f);
                o.Set(x, y, (byte)(10 + 40 * v), (byte)(16 + 220 * v), (byte)(28 + 225 * v));
            }
        return o;
    }

    static readonly (float T, byte R, byte G, byte B)[] Ironbow =
    {
        (0f, 8, 4, 40), (0.25f, 80, 10, 140), (0.5f, 200, 40, 90), (0.72f, 250, 130, 20), (0.9f, 255, 220, 80), (1f, 255, 255, 230),
    };

    static (byte, byte, byte) Ramp((float T, byte R, byte G, byte B)[] ramp, float v)
    {
        v = Math.Clamp(v, 0, 1);
        for (int i = 1; i < ramp.Length; i++)
            if (v <= ramp[i].T)
            {
                var a = ramp[i - 1]; var b = ramp[i];
                float k = (v - a.T) / (b.T - a.T);
                return ((byte)(a.R + (b.R - a.R) * k), (byte)(a.G + (b.G - a.G) * k), (byte)(a.B + (b.B - a.B) * k));
            }
        return (ramp[^1].R, ramp[^1].G, ramp[^1].B);
    }

    /// <summary>Pseudo-thermal: vegetation/water cool, asphalt & roofs warm, targets (engines/bodies) hot.</summary>
    public static RgbImage ThermalView(RgbImage img)
    {
        var red = RedMask(img);
        var heat = new GrayImage(img.Width, img.Height);
        for (int y = 0; y < img.Height; y++)
            for (int x = 0; x < img.Width; x++)
            {
                var (r, g, b) = img.Get(x, y);
                float veg = (g - r) / 255f, water = (b - r) / 255f;
                float lum = (r + g + b) / 765f;
                heat[x, y] = red[y * img.Width + x] ? 1f : Math.Clamp(0.18f + 0.38f * lum - 0.9f * veg - 0.8f * water, 0.04f, 0.62f);
            }
        heat = GaussianBlur3(GaussianBlur3(heat));
        var o = new RgbImage(img.Width, img.Height);
        for (int y = 0; y < img.Height; y++)
            for (int x = 0; x < img.Width; x++)
            {
                var (r, g, b) = Ramp(Ironbow, heat[x, y]);
                o.Set(x, y, r, g, b);
            }
        return o;
    }

    static readonly (float T, byte R, byte G, byte B)[] NdviRamp =
    {
        (0f, 120, 40, 30), (0.35f, 200, 160, 80), (0.55f, 210, 210, 90), (0.75f, 90, 180, 60), (1f, 20, 110, 40),
    };

    /// <summary>Visible-band vegetation index (VARI-like) for crop-health style mapping.</summary>
    public static RgbImage VegetationView(RgbImage img)
    {
        var o = new RgbImage(img.Width, img.Height);
        for (int y = 0; y < img.Height; y++)
            for (int x = 0; x < img.Width; x++)
            {
                var (r, g, b) = img.Get(x, y);
                float denom = g + r - b;
                float vari = Math.Abs(denom) < 1 ? 0 : (g - r) / denom;
                var (rr, gg, bb) = Ramp(NdviRamp, Math.Clamp(vari * 1.6f + 0.45f, 0, 1));
                o.Set(x, y, rr, gg, bb);
            }
        return o;
    }

    public static RgbImage Apply(RgbImage img, VisionMode mode) => mode switch
    {
        VisionMode.Edges => EdgeView(img),
        VisionMode.Thermal => ThermalView(img),
        VisionMode.Vegetation => VegetationView(img),
        _ => img,
    };
}
