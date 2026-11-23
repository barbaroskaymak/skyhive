using SkiaSharp;

namespace SkyHive.App.Views;

public static class Palette
{
    public static readonly SKColor Bg = SKColor.Parse("#0A0F1C");
    public static readonly SKColor Panel = SKColor.Parse("#111A2E");
    public static readonly SKColor Accent = SKColor.Parse("#22D3EE");
    public static readonly SKColor Good = SKColor.Parse("#34D399");
    public static readonly SKColor Warn = SKColor.Parse("#FBBF24");
    public static readonly SKColor Danger = SKColor.Parse("#F43F5E");
    public static readonly SKColor Text = SKColor.Parse("#E2E8F0");
    public static readonly SKColor Muted = SKColor.Parse("#7C8AA5");

    static readonly string[] DroneHex = { "#22D3EE", "#FBBF24", "#F472B6", "#A3E635", "#A78BFA", "#FB923C", "#38BDF8", "#34D399" };
    public static SKColor Drone(int index) => SKColor.Parse(DroneHex[index % DroneHex.Length]);
    public static Color DroneMaui(int index) => Color.FromArgb(DroneHex[index % DroneHex.Length]);

    public static SKFont Font(float size, bool bold = false) =>
        new(SKTypeface.FromFamilyName(null, bold ? SKFontStyle.Bold : SKFontStyle.Normal), size);
}
