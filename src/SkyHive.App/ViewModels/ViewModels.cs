using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using SkyHive.App.Views;
using SkyHive.Core;
using SkyHive.Core.Geo;
using SkyHive.Core.Model;
using SkyHive.Core.Swarm;
using SkyHive.Core.Vision;

namespace SkyHive.App.ViewModels;

public abstract class ObservableObject : INotifyPropertyChanged
{
    public event PropertyChangedEventHandler? PropertyChanged;

    protected bool Set<T>(ref T field, T value, [CallerMemberName] string? name = null)
    {
        if (EqualityComparer<T>.Default.Equals(field, value)) return false;
        field = value;
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        return true;
    }

    protected void Raise([CallerMemberName] string? name = null) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}

public sealed class DroneItemViewModel(DroneAgent agent) : ObservableObject
{
    string _mode = "", _task = "", _telemetry = "", _battery = "", _gps = "";
    double _batteryLevel;
    Color _statusColor = Colors.Gray, _batteryColor = Colors.Gray, _modeColor = Colors.Gray;
    bool _selected;

    public DroneAgent Agent { get; } = agent;
    public int Id => Agent.Id;
    public string Name => Agent.Name;
    public string Endpoint => Agent.Link.Endpoint;
    public Color Accent { get; } = Palette.DroneMaui(agent.Index);
    public string Mode { get => _mode; private set => Set(ref _mode, value); }
    public Color ModeColor { get => _modeColor; private set => Set(ref _modeColor, value); }
    public string Task { get => _task; private set => Set(ref _task, value); }
    public string Telemetry { get => _telemetry; private set => Set(ref _telemetry, value); }
    public string Battery { get => _battery; private set => Set(ref _battery, value); }
    public string Gps { get => _gps; private set => Set(ref _gps, value); }
    public double BatteryLevel { get => _batteryLevel; private set => Set(ref _batteryLevel, value); }
    public Color BatteryColor { get => _batteryColor; private set => Set(ref _batteryColor, value); }
    public Color StatusColor { get => _statusColor; private set => Set(ref _statusColor, value); }
    public bool IsSelected { get => _selected; set { if (Set(ref _selected, value)) Raise(nameof(CardStroke)); } }
    public Color CardStroke => IsSelected ? Accent : Color.FromArgb("#1E2A44");

    public void Refresh()
    {
        var s = Agent.State;
        Mode = s.Connected ? s.ModeName + (s.Armed ? " · ARMED" : "") : "BAĞLANTI YOK";
        ModeColor = !s.Connected ? Color.FromArgb("#64748B") : s.Armed ? Color.FromArgb("#34D399") : Color.FromArgb("#94A3B8");
        Task = Agent.TaskName;
        var dist = Agent.Local.Length;
        Telemetry = $"{s.Position.Alt,5:F1} m   {s.GroundSpeed,4:F1} m/s   {dist,4:F0} m";
        var pct = Math.Max(0, s.BatteryPercent);
        Battery = s.BatteryPercent < 0 ? "—" : $"{pct:F0}%  {s.BatteryVoltage:F1}V";
        BatteryLevel = pct / 100.0;
        BatteryColor = pct > 50 ? Color.FromArgb("#34D399") : pct > 25 ? Color.FromArgb("#FBBF24") : Color.FromArgb("#F43F5E");
        Gps = s.GpsFix >= 3 ? $"GPS 3D · {s.Satellites} uydu" : "GPS yok";
        StatusColor = !s.Connected ? Color.FromArgb("#F43F5E") : s.Airborne ? Color.FromArgb("#22D3EE") : Color.FromArgb("#34D399");
    }
}

public sealed class TargetItemViewModel : ObservableObject
{
    string _detail = "";
    public required int Id { get; init; }
    public string Label => $"T{Id}";
    public string Detail { get => _detail; set => Set(ref _detail, value); }
    Color _badge = Colors.Orange;
    public Color Badge { get => _badge; set => Set(ref _badge, value); }
}

public sealed record LogItemViewModel(string Time, string Source, string Message, Color Color);

public sealed class MainViewModel : ObservableObject
{
    SwarmSession? _session;
    bool _connected, _busy;
    int _modeIndex, _droneCount = 5;
    string _host = "127.0.0.1", _basePort = "5760", _command = "";
    string _airborne = "0/0", _targets = "0", _avgBattery = "—", _linkStatus = "Bağlı değil";
    DroneItemViewModel? _selected;
    VisionMode _cameraMode = VisionMode.Detection;

    public ObservableCollection<DroneItemViewModel> Drones { get; } = new();
    public ObservableCollection<TargetItemViewModel> Targets { get; } = new();
    public ObservableCollection<LogItemViewModel> Log { get; } = new();
    public IReadOnlyList<string> ModeOptions { get; } = new[] { "Dahili Simülasyon", "ArduPilot SITL (TCP)", "ArduPilot SITL (UDP)" };

    public SwarmSession? Session { get => _session; private set => Set(ref _session, value); }
    public bool IsConnected { get => _connected; private set { if (Set(ref _connected, value)) { Raise(nameof(ConnectText)); Raise(nameof(IsDisconnected)); } } }
    public bool IsDisconnected => !IsConnected;
    public bool IsBusy { get => _busy; private set => Set(ref _busy, value); }
    public string ConnectText => IsConnected ? "BAĞLANTIYI KES" : "BAĞLAN";
    public int ModeIndex { get => _modeIndex; set { if (Set(ref _modeIndex, value)) { Raise(nameof(IsSitl)); BasePort = value == 2 ? "14550" : "5760"; } } }
    public bool IsSitl => ModeIndex > 0;
    public int DroneCount { get => _droneCount; set { if (Set(ref _droneCount, Math.Clamp(value, 1, 12))) Raise(nameof(DroneCountText)); } }
    public string DroneCountText => $"{DroneCount} drone";
    public string Host { get => _host; set => Set(ref _host, value); }
    public string BasePort { get => _basePort; set => Set(ref _basePort, value); }
    public string CommandText { get => _command; set => Set(ref _command, value); }
    public string AirborneStat { get => _airborne; private set => Set(ref _airborne, value); }
    public string TargetStat { get => _targets; private set => Set(ref _targets, value); }
    public string BatteryStat { get => _avgBattery; private set => Set(ref _avgBattery, value); }
    public string LinkStat { get => _linkStatus; private set => Set(ref _linkStatus, value); }
    public VisionMode CameraMode { get => _cameraMode; set => Set(ref _cameraMode, value); }

    public DroneItemViewModel? Selected
    {
        get => _selected;
        set
        {
            if (_selected is not null) _selected.IsSelected = false;
            if (Set(ref _selected, value) && value is not null) value.IsSelected = true;
            Raise(nameof(CameraTitle));
        }
    }

    public string CameraTitle => Selected is null ? "KAMERA" : $"KAMERA · {Selected.Name}";

    public async Task ToggleConnectionAsync()
    {
        if (IsBusy) return;
        IsBusy = true;
        try
        {
            if (Session is not null)
            {
                var s = Session;
                Session = null;
                IsConnected = false;
                Drones.Clear();
                Targets.Clear();
                Selected = null;
                await s.DisposeAsync();
                AddLog(new LogEntry(DateTime.Now, LogLevel.Info, "SkyHive", "Bağlantı kapatıldı"));
                return;
            }
            var options = new SessionOptions
            {
                Mode = (LinkMode)ModeIndex,
                DroneCount = DroneCount,
                Host = string.IsNullOrWhiteSpace(Host) ? "127.0.0.1" : Host.Trim(),
                BasePort = int.TryParse(BasePort, out var p) ? p : 5760,
                PortStride = 10,
            };
            var session = await SwarmSession.StartAsync(options);
            foreach (var e in session.LogSnapshot()) AddLog(e);
            session.Logged += e => MainThread.BeginInvokeOnMainThread(() => AddLog(e));
            foreach (var a in session.Agents) Drones.Add(new DroneItemViewModel(a));
            Selected = Drones.FirstOrDefault();
            Session = session;
            IsConnected = true;
        }
        catch (Exception ex)
        {
            AddLog(new LogEntry(DateTime.Now, LogLevel.Error, "SkyHive", ex.Message));
        }
        finally { IsBusy = false; }
    }

    public async Task ExecuteAsync(string? text = null)
    {
        var cmd = (text ?? CommandText).Trim();
        if (cmd.Length == 0) return;
        if (Session is null) { AddLog(new LogEntry(DateTime.Now, LogLevel.Warning, "Konsol", "Önce bağlanın")); return; }
        if (text is null) CommandText = "";
        AddLog(new LogEntry(DateTime.Now, LogLevel.Info, "›", cmd));
        var r = await Session.Commands.ExecuteAsync(cmd, Selected?.Id);
        foreach (var line in r.Message.Split('\n'))
            AddLog(new LogEntry(DateTime.Now, r.Ok ? LogLevel.Success : LogLevel.Error, "Konsol", line));
    }

    public void GoTo(Vec2 point)
    {
        if (Session is null || Selected is null) return;
        Session.Controller.Goto(Selected.Id, point);
        AddLog(new LogEntry(DateTime.Now, LogLevel.Info, Selected.Name, $"Dokun-git → {point}"));
    }

    void AddLog(LogEntry e)
    {
        var color = e.Level switch
        {
            LogLevel.Success => Color.FromArgb("#34D399"),
            LogLevel.Warning => Color.FromArgb("#FBBF24"),
            LogLevel.Error => Color.FromArgb("#F43F5E"),
            _ => e.Source == "›" ? Color.FromArgb("#22D3EE") : Color.FromArgb("#CBD5E1"),
        };
        Log.Add(new LogItemViewModel(e.Time.ToString("HH:mm:ss"), e.Source, e.Message, color));
        while (Log.Count > 300) Log.RemoveAt(0);
    }

    public void Refresh()
    {
        var s = Session;
        if (s is null) return;
        foreach (var d in Drones) d.Refresh();
        int air = s.Agents.Count(a => a.State.Airborne), conn = s.Agents.Count(a => a.State.Connected);
        AirborneStat = $"{air}/{s.Agents.Count}";
        LinkStat = $"{conn}/{s.Agents.Count} bağlı";
        var bats = s.Agents.Where(a => a.State.BatteryPercent >= 0).Select(a => a.State.BatteryPercent).ToList();
        BatteryStat = bats.Count == 0 ? "—" : $"{bats.Average():F0}%";

        var tracks = s.Tracker.Snapshot();
        TargetStat = tracks.Count(t => t.Confirmed).ToString();
        foreach (var t in tracks)
        {
            var vm = Targets.FirstOrDefault(x => x.Id == t.Id);
            if (vm is null) { vm = new TargetItemViewModel { Id = t.Id }; Targets.Insert(0, vm); }
            var age = (DateTime.UtcNow - t.LastSeen).TotalSeconds;
            vm.Detail = $"N{t.Position.N:F0} E{t.Position.E:F0} · {t.Hits} görüş · {(t.Velocity.Length > 0.6 ? $"{t.Velocity.Length:F1} m/s" : "sabit")} · {t.LastSeenBy} · {age:F0}s";
            vm.Badge = t.Confirmed ? Color.FromArgb("#F43F5E") : Color.FromArgb("#FBBF24");
        }
        foreach (var gone in Targets.Where(x => tracks.All(t => t.Id != x.Id)).ToList()) Targets.Remove(gone);
    }
}
