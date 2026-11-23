using SkyHive.Core.Geo;

namespace SkyHive.Core.Model;

/// <summary>ArduCopter flight modes (values are the ArduCopter custom_mode numbers).</summary>
public enum FlightMode
{
    Stabilize = 0,
    Acro = 1,
    AltHold = 2,
    Auto = 3,
    Guided = 4,
    Loiter = 5,
    Rtl = 6,
    Circle = 7,
    Land = 9,
    Drift = 11,
    Sport = 13,
    PosHold = 16,
    Brake = 17,
    SmartRtl = 21,
    Unknown = -1,
}

public enum LogLevel { Info, Success, Warning, Error }

public sealed record LogEntry(DateTime Time, LogLevel Level, string Source, string Message);

/// <summary>Live telemetry of one vehicle. Written by the link, read by the UI.</summary>
public sealed class DroneState
{
    public bool Connected { get; set; }
    public bool Armed { get; set; }
    public FlightMode Mode { get; set; } = FlightMode.Unknown;
    public uint RawMode { get; set; }
    public GeoPoint Position { get; set; }
    public GeoPoint? Home { get; set; }
    public bool HasPosition { get; set; }
    public double Heading { get; set; }
    public double GroundSpeed { get; set; }
    public double ClimbRate { get; set; }
    public double Roll { get; set; }
    public double Pitch { get; set; }
    public double BatteryPercent { get; set; } = -1;
    public double BatteryVoltage { get; set; }
    public int GpsFix { get; set; }
    public int Satellites { get; set; }
    public DateTime LastHeartbeat { get; set; }
    public string LastStatusText { get; set; } = "";

    public bool Airborne => Armed && Position.Alt > 0.8;
    public string ModeName => Mode == FlightMode.Unknown ? $"MODE {RawMode}" : Mode switch
    {
        FlightMode.AltHold => "ALT_HOLD",
        FlightMode.Rtl => "RTL",
        FlightMode.SmartRtl => "SMART_RTL",
        FlightMode.PosHold => "POSHOLD",
        _ => Mode.ToString().ToUpperInvariant(),
    };
}
