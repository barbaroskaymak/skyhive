using SkyHive.Core;
using SkyHive.Core.Model;

// SkyHive headless console: drive the swarm from a terminal (built-in sim or ArduPilot SITL).
//   dotnet run --project tools/SkyHive.Cli -- --sitl --count 3            (interactive)
//   dotnet run --project tools/SkyHive.Cli -- --sitl --count 3 --script "kalk 15; bekle 25; formasyon v 10; git d1 60 40; bekle 30; eve dön; bekle 40"
var argList = args.ToList();
string? Arg(string name) { var i = argList.IndexOf(name); return i >= 0 && i + 1 < argList.Count ? argList[i + 1] : null; }

var options = new SessionOptions
{
    Mode = argList.Contains("--udp") ? LinkMode.SitlUdp : argList.Contains("--sitl") ? LinkMode.SitlTcp : LinkMode.Simulation,
    DroneCount = int.TryParse(Arg("--count"), out var n) ? n : 3,
    Host = Arg("--host") ?? "127.0.0.1",
    BasePort = int.TryParse(Arg("--port"), out var p) ? p : (argList.Contains("--udp") ? 14550 : 5760),
};

await using var session = await SwarmSession.StartAsync(options);
session.Logged += e =>
{
    Console.ForegroundColor = e.Level switch { LogLevel.Success => ConsoleColor.Green, LogLevel.Warning => ConsoleColor.Yellow, LogLevel.Error => ConsoleColor.Red, _ => ConsoleColor.Gray };
    Console.WriteLine($"{e.Time:HH:mm:ss} [{e.Source}] {e.Message}");
    Console.ResetColor();
};

void PrintStatus()
{
    foreach (var a in session.Agents)
    {
        var s = a.State;
        Console.WriteLine($"  {a.Name} {(s.Connected ? "●" : "○")} {s.ModeName,-9} {(s.Armed ? "ARMED " : "      ")} alt {s.Position.Alt,6:F1} m  gs {s.GroundSpeed,4:F1}  pos {a.Local}  bat {s.BatteryPercent,3:F0}%  gps {s.GpsFix}/{s.Satellites}  task: {a.TaskName}");
    }
    var t = session.Tracker.Snapshot();
    if (t.Count > 0) Console.WriteLine($"  hedefler: {string.Join(", ", t.Select(x => $"{x.Label}{(x.Confirmed ? "✔" : "?")}"))}");
}

Console.WriteLine($"SkyHive CLI — {options.Mode}, {options.DroneCount} drone. Bağlantı bekleniyor…");
var deadline = DateTime.UtcNow.AddSeconds(60);
while (!session.Agents.All(a => a.State.Connected && a.State.HasPosition) && DateTime.UtcNow < deadline) await Task.Delay(500);
PrintStatus();

var script = Arg("--script");
if (script is not null)
{
    foreach (var raw in script.Split(';', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
    {
        if (raw.StartsWith("bekle ") && double.TryParse(raw[6..], out var secs))
        {
            for (int i = 0; i < secs; i += 5) { await Task.Delay(TimeSpan.FromSeconds(Math.Min(5, secs - i))); PrintStatus(); }
            continue;
        }
        Console.WriteLine($"> {raw}");
        var r = await session.Commands.ExecuteAsync(raw);
        Console.WriteLine((r.Ok ? "  ✔ " : "  ✖ ") + r.Message);
    }
    return;
}

Console.WriteLine("Komut yazın ('yardım', 'durum', 'çık').");
while (Console.ReadLine() is { } line)
{
    if (line.Trim() is "çık" or "cik" or "exit") break;
    if (line.Trim() is "durum" or "status") { PrintStatus(); continue; }
    var r = await session.Commands.ExecuteAsync(line);
    Console.WriteLine((r.Ok ? "✔ " : "✖ ") + r.Message);
}
