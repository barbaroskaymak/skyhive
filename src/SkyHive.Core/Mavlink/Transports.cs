using System.Net;
using System.Net.Sockets;

namespace SkyHive.Core.Mavlink;

/// <summary>Byte transport for a MAVLink stream (TCP client or UDP).</summary>
public interface IMavTransport : IAsyncDisposable
{
    string Description { get; }
    bool IsOpen { get; }
    Task OpenAsync(CancellationToken ct);
    Task SendAsync(byte[] data, CancellationToken ct);
    /// <summary>Reads the next chunk; returns 0 when the connection dropped.</summary>
    Task<int> ReceiveAsync(byte[] buffer, CancellationToken ct);
}

/// <summary>Connects to an ArduPilot SITL serial port exposed as TCP (5760 + 10 × instance).</summary>
public sealed class TcpMavTransport(string host, int port) : IMavTransport
{
    TcpClient? _client;
    NetworkStream? _stream;
    public string Description => $"tcp:{host}:{port}";
    public bool IsOpen => _client?.Connected == true;

    public async Task OpenAsync(CancellationToken ct)
    {
        _client?.Dispose();
        _client = new TcpClient { NoDelay = true };
        await _client.ConnectAsync(host, port, ct);
        _stream = _client.GetStream();
    }

    public async Task SendAsync(byte[] data, CancellationToken ct)
    {
        if (_stream is null) return;
        await _stream.WriteAsync(data, ct);
    }

    public async Task<int> ReceiveAsync(byte[] buffer, CancellationToken ct) =>
        _stream is null ? 0 : await _stream.ReadAsync(buffer, ct);

    public ValueTask DisposeAsync()
    {
        _client?.Dispose();
        return ValueTask.CompletedTask;
    }
}

/// <summary>Listens on a local UDP port (MAVProxy / SITL "--out udp:host:port"); replies go to the last sender.</summary>
public sealed class UdpMavTransport(int localPort, string? remoteHost = null, int remotePort = 0) : IMavTransport
{
    UdpClient? _udp;
    IPEndPoint? _remote;
    public string Description => $"udp:{localPort}";
    public bool IsOpen => _udp is not null;

    public Task OpenAsync(CancellationToken ct)
    {
        _udp = new UdpClient(new IPEndPoint(IPAddress.Any, localPort));
        if (remoteHost is not null && remotePort > 0)
            _remote = new IPEndPoint(Dns.GetHostAddresses(remoteHost)[0], remotePort);
        return Task.CompletedTask;
    }

    public async Task SendAsync(byte[] data, CancellationToken ct)
    {
        if (_udp is null || _remote is null) return;
        await _udp.SendAsync(data, _remote, ct);
    }

    public async Task<int> ReceiveAsync(byte[] buffer, CancellationToken ct)
    {
        if (_udp is null) return 0;
        var r = await _udp.ReceiveAsync(ct);
        _remote = r.RemoteEndPoint;
        r.Buffer.CopyTo(buffer, 0);
        return r.Buffer.Length;
    }

    public ValueTask DisposeAsync()
    {
        _udp?.Dispose();
        return ValueTask.CompletedTask;
    }
}
