namespace SkyHive.App;

public partial class App : Application
{
    public App()
    {
        InitializeComponent();
        UserAppTheme = AppTheme.Dark;
    }

    protected override Window CreateWindow(IActivationState? activationState) => new(new MainPage())
    {
        Title = "SkyHive — Sürü Drone Komuta Merkezi",
        Width = 1560,
        Height = 980,
        MinimumWidth = 1100,
        MinimumHeight = 720,
    };
}
