using SkyHive.Core.Geo;
using SkyHive.Core.Model;

namespace SkyHive.Core.Links;

/// <summary>Command/telemetry channel to a single vehicle (built-in simulator or ArduPilot over MAVLink).</summary>
public interface IDroneLink : IAsyncDisposable
{
    int Id { get; }
    string Name { get; }
    string Endpoint { get; }
    DroneState State { get; }
    event Action<IDroneLink, LogLevel, string>? Log;

    Task StartAsync(CancellationToken ct);
    Task<bool> SetModeAsync(FlightMode mode);
    Task<bool> ArmAsync(bool arm);
    /// <summary>Full takeoff sequence: GUIDED → arm → NAV_TAKEOFF.</summary>
    Task<bool> TakeoffAsync(double altitude);
    /// <summary>Guided-mode position target (relative altitude).</summary>
    Task GotoAsync(GeoPoint target);
    Task<bool> LandAsync();
    Task<bool> ReturnToLaunchAsync();
    Task SetSpeedAsync(double metresPerSecond);
}
