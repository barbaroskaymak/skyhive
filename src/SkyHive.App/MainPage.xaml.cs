using SkyHive.App.ViewModels;
using SkyHive.Core;
using SkyHive.Core.Vision;

namespace SkyHive.App;

public partial class MainPage : ContentPage
{
    readonly MainViewModel _vm = new();
    readonly IDispatcherTimer _timer;
    SwarmSession? _boundSession;
    bool _capturing, _narrow, _layoutInitialised;
    int _tick;
    string _tab = "ops";

    static readonly Color ChipOn = Color.FromArgb("#0E3A47");
    static readonly Color ChipOff = Color.FromArgb("#131D33");
    static readonly Color Accent = Color.FromArgb("#22D3EE");
    static readonly Color Stroke = Color.FromArgb("#1E2A44");

    public MainPage()
    {
        InitializeComponent();
        BindingContext = _vm;
        _vm.PropertyChanged += (_, e) =>
        {
            if (e.PropertyName == nameof(MainViewModel.Selected)) Map.SelectedId = _vm.Selected?.Id;
        };
        Map.DroneTapped += id => _vm.Selected = _vm.Drones.FirstOrDefault(d => d.Id == id);
        Map.GroundTapped += p => _vm.GoTo(p);
        UpdateChips();

        _timer = Dispatcher.CreateTimer();
        _timer.Interval = TimeSpan.FromMilliseconds(40);
        _timer.Tick += (_, _) => OnTick();
        _timer.Start();
        Loaded += OnLoaded;
    }

    async void OnLoaded(object? sender, EventArgs e)
    {
        // "--demo" (or SKYHIVE_DEMO=1): auto-connect to the built-in simulator and run the autonomous showcase.
        var args = Environment.GetCommandLineArgs();
        if (args.FirstOrDefault(a => a.StartsWith("--camera=")) is { } cm && Enum.TryParse<VisionMode>(cm[9..], true, out var vm))
        {
            _vm.CameraMode = vm;
            UpdateChips();
        }
        if (args.Contains("--demo") || Environment.GetEnvironmentVariable("SKYHIVE_DEMO") == "1")
        {
            await _vm.ToggleConnectionAsync();
            await _vm.ExecuteAsync("demo");
        }
    }

    void OnTick()
    {
        _tick++;
        if (!ReferenceEquals(_boundSession, _vm.Session))
        {
            _boundSession = _vm.Session;
            Map.Session = _boundSession;
            if (_boundSession is not null)
            {
                Map.LoadTerrain(_boundSession.World);
                Map.CenterOnSwarm();
            }
            else Camera.SetFrame(null);
        }
        Map.InvalidateSurface();
        if (_tick % 6 == 0) _vm.Refresh();
        if (_tick % 3 == 0) _ = CaptureAsync();
    }

    async Task CaptureAsync()
    {
        var session = _vm.Session;
        var selected = _vm.Selected;
        if (_capturing || session is null || selected is null) return;
        _capturing = true;
        try
        {
            var mode = _vm.CameraMode;
            var agent = selected.Agent;
            Camera.Mode = mode;
            Camera.ColorIndex = agent.Index;
            if (!agent.State.HasPosition) { Camera.SetFrame(null); return; }
            var frame = await Task.Run(() => session.Vision.Capture(agent, mode));
            Camera.SetFrame(frame);
            VisionStat.Text = $"{session.Vision.FramesProcessed} kare · {session.Vision.LastProcessMs:F0} ms";
        }
        catch { /* frame dropped */ }
        finally { _capturing = false; }
    }

    // ------------------------------------------------------------ events

    async void OnConnect(object? sender, EventArgs e) => await _vm.ToggleConnectionAsync();
    void OnCountMinus(object? sender, EventArgs e) => _vm.DroneCount--;
    void OnCountPlus(object? sender, EventArgs e) => _vm.DroneCount++;

    async void OnSend(object? sender, EventArgs e)
    {
        await _vm.ExecuteAsync();
        CommandEntry.Focus();
    }

    async void OnQuick(object? sender, EventArgs e)
    {
        if (sender is Button { ClassId: { } cmd }) await _vm.ExecuteAsync(cmd);
    }

    async void OnTargetTapped(object? sender, TappedEventArgs e)
    {
        if (e.Parameter is int id) await _vm.ExecuteAsync($"takip t{id}");
    }

    void OnCameraMode(object? sender, EventArgs e)
    {
        if (sender is Button { ClassId: { } m } && Enum.TryParse<VisionMode>(m, out var mode))
        {
            _vm.CameraMode = mode;
            UpdateChips();
        }
    }

    void UpdateChips()
    {
        foreach (var b in ModeChips.Children.OfType<Button>())
        {
            bool on = b.ClassId == _vm.CameraMode.ToString();
            b.BackgroundColor = on ? ChipOn : ChipOff;
            b.BorderColor = on ? Accent : Stroke;
            b.TextColor = on ? Accent : Color.FromArgb("#E2E8F0");
        }
        Toggle(FollowButton, Map.FollowSelected);
        Toggle(TapGoButton, Map.TapToGo);
    }

    static void Toggle(Button b, bool on)
    {
        b.BackgroundColor = on ? ChipOn : ChipOff;
        b.BorderColor = on ? Accent : Stroke;
        b.TextColor = on ? Accent : Color.FromArgb("#E2E8F0");
    }

    void OnZoomIn(object? sender, EventArgs e) => Map.ZoomBy(1.35);
    void OnZoomOut(object? sender, EventArgs e) => Map.ZoomBy(1 / 1.35);
    void OnCenter(object? sender, EventArgs e) => Map.CenterOnSwarm();
    void OnFollow(object? sender, EventArgs e) { Map.FollowSelected = !Map.FollowSelected; UpdateChips(); }
    void OnTapGo(object? sender, EventArgs e) { Map.TapToGo = !Map.TapToGo; UpdateChips(); }

    // ------------------------------------------------------- responsive layout

    protected override void OnSizeAllocated(double width, double height)
    {
        base.OnSizeAllocated(width, height);
        bool narrow = width < 1000;
        if (_layoutInitialised && narrow == _narrow) return;
        _layoutInitialised = true;
        _narrow = narrow;
        ApplyLayout();
    }

    void OnTab(object? sender, EventArgs e)
    {
        if (sender is Button { ClassId: { } t }) { _tab = t; ApplyLayout(); }
    }

    void ApplyLayout()
    {
        Root.RowDefinitions.Clear();
        Root.ColumnDefinitions.Clear();
        if (!_narrow)
        {
            Root.RowDefinitions.Add(new RowDefinition(GridLength.Auto));
            Root.RowDefinitions.Add(new RowDefinition(GridLength.Star));
            Root.RowDefinitions.Add(new RowDefinition(new GridLength(210)));
            Root.ColumnDefinitions.Add(new ColumnDefinition(new GridLength(300)));
            Root.ColumnDefinitions.Add(new ColumnDefinition(GridLength.Star));
            Root.ColumnDefinitions.Add(new ColumnDefinition(new GridLength(390)));
            TabBar.IsVisible = false;
            Place(HeaderPanel, 0, 0, 1, 3);
            Place(FleetPanel, 1, 0, 2, 1);
            Place(MapPanel, 1, 1, 1, 1);
            Place(ConsolePanel, 2, 1, 1, 1);
            Place(RightPanel, 1, 2, 2, 1);
            FleetPanel.IsVisible = MapPanel.IsVisible = ConsolePanel.IsVisible = RightPanel.IsVisible = true;
            Root.Padding = new Thickness(14);
            return;
        }
        // phone / small tablet: header, map, tab bar, active panel
        Root.RowDefinitions.Add(new RowDefinition(GridLength.Auto));
        Root.RowDefinitions.Add(new RowDefinition(new GridLength(Math.Max(300, Height * 0.42))));
        Root.RowDefinitions.Add(new RowDefinition(GridLength.Auto));
        Root.RowDefinitions.Add(new RowDefinition(GridLength.Star));
        Root.ColumnDefinitions.Add(new ColumnDefinition(GridLength.Star));
        Root.Padding = new Thickness(8);
        TabBar.IsVisible = true;
        Place(HeaderPanel, 0, 0, 1, 1);
        Place(MapPanel, 1, 0, 1, 1);
        Place(TabBar, 2, 0, 1, 1);
        Place(FleetPanel, 3, 0, 1, 1);
        Place(RightPanel, 3, 0, 1, 1);
        Place(ConsolePanel, 3, 0, 1, 1);
        FleetPanel.IsVisible = _tab == "fleet";
        RightPanel.IsVisible = _tab == "ops";
        ConsolePanel.IsVisible = _tab == "console";
        foreach (var b in TabBar.Children.OfType<Button>()) Toggle(b, b.ClassId == _tab);
    }

    static void Place(View v, int row, int col, int rowSpan, int colSpan)
    {
        Grid.SetRow(v, row);
        Grid.SetColumn(v, col);
        Grid.SetRowSpan(v, rowSpan);
        Grid.SetColumnSpan(v, colSpan);
    }
}
