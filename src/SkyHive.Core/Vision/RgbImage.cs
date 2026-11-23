namespace SkyHive.Core.Vision;

/// <summary>Simple RGBA8888 image buffer (row-major, 4 bytes per pixel).</summary>
public sealed class RgbImage(int width, int height)
{
    public int Width { get; } = width;
    public int Height { get; } = height;
    public byte[] Pixels { get; } = new byte[width * height * 4];

    public void Set(int x, int y, byte r, byte g, byte b)
    {
        int i = (y * Width + x) * 4;
        Pixels[i] = r; Pixels[i + 1] = g; Pixels[i + 2] = b; Pixels[i + 3] = 255;
    }

    public (byte R, byte G, byte B) Get(int x, int y)
    {
        int i = (y * Width + x) * 4;
        return (Pixels[i], Pixels[i + 1], Pixels[i + 2]);
    }

    public RgbImage Clone()
    {
        var c = new RgbImage(Width, Height);
        Pixels.CopyTo(c.Pixels, 0);
        return c;
    }
}

/// <summary>Single-channel float image.</summary>
public sealed class GrayImage(int width, int height)
{
    public int Width { get; } = width;
    public int Height { get; } = height;
    public float[] Data { get; } = new float[width * height];
    public float this[int x, int y]
    {
        get => Data[y * Width + x];
        set => Data[y * Width + x] = value;
    }
}
