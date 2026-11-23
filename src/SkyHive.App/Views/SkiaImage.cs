using System.Runtime.InteropServices;
using SkiaSharp;
using SkyHive.Core.Vision;

namespace SkyHive.App.Views;

public static class SkiaImage
{
    public static SKBitmap ToBitmap(RgbImage img)
    {
        var bmp = new SKBitmap(new SKImageInfo(img.Width, img.Height, SKColorType.Rgba8888, SKAlphaType.Premul));
        Marshal.Copy(img.Pixels, 0, bmp.GetPixels(), img.Pixels.Length);
        bmp.NotifyPixelsChanged();
        return bmp;
    }
}
