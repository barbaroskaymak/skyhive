using System.Buffers.Binary;

namespace SkyHive.Core.Mavlink;

/// <summary>X.25 / MCRF4XX checksum used by MAVLink.</summary>
public static class MavCrc
{
    public const ushort Init = 0xFFFF;

    public static ushort Accumulate(byte b, ushort crc)
    {
        byte tmp = (byte)(b ^ (byte)(crc & 0xFF));
        tmp ^= (byte)(tmp << 4);
        return (ushort)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
    }

    public static ushort Accumulate(ReadOnlySpan<byte> data, ushort crc)
    {
        foreach (var b in data) crc = Accumulate(b, crc);
        return crc;
    }
}

/// <summary>Message ids and CRC_EXTRA seeds (common + ardupilotmega dialect, verified against pymavlink).</summary>
public static class MavMsg
{
    public const uint Heartbeat = 0;
    public const uint SysStatus = 1;
    public const uint SetMode = 11;
    public const uint GpsRawInt = 24;
    public const uint Attitude = 30;
    public const uint GlobalPositionInt = 33;
    public const uint MissionCurrent = 42;
    public const uint RequestDataStream = 66;
    public const uint VfrHud = 74;
    public const uint CommandLong = 76;
    public const uint CommandAck = 77;
    public const uint SetPositionTargetGlobalInt = 86;
    public const uint BatteryStatus = 147;
    public const uint HomePosition = 242;
    public const uint StatusText = 253;

    public static readonly IReadOnlyDictionary<uint, byte> CrcExtra = new Dictionary<uint, byte>
    {
        [Heartbeat] = 50, [SysStatus] = 124, [SetMode] = 89, [GpsRawInt] = 24, [Attitude] = 39,
        [GlobalPositionInt] = 104, [MissionCurrent] = 28, [RequestDataStream] = 148, [VfrHud] = 20,
        [CommandLong] = 152, [CommandAck] = 143, [SetPositionTargetGlobalInt] = 5,
        [BatteryStatus] = 154, [HomePosition] = 104, [StatusText] = 83,
    };
}

public static class MavCmd
{
    public const ushort NavReturnToLaunch = 20;
    public const ushort NavLand = 21;
    public const ushort NavTakeoff = 22;
    public const ushort ConditionYaw = 115;
    public const ushort DoSetMode = 176;
    public const ushort DoChangeSpeed = 178;
    public const ushort ComponentArmDisarm = 400;
    public const ushort GetHomePosition = 410;
    public const ushort SetMessageInterval = 511;
}

public enum MavResult : byte
{
    Accepted = 0, TemporarilyRejected = 1, Denied = 2, Unsupported = 3, Failed = 4, InProgress = 5, Cancelled = 6,
}

public sealed record MavlinkFrame(byte Seq, byte SysId, byte CompId, uint MsgId, byte[] Payload, bool IsV2)
{
    /// <summary>Payload padded with zeros (MAVLink 2 truncates trailing zero bytes).</summary>
    public ReadOnlySpan<byte> Body(int minLength)
    {
        if (Payload.Length >= minLength) return Payload;
        var padded = new byte[minLength];
        Payload.CopyTo(padded, 0);
        return padded;
    }
}

/// <summary>Incremental parser for a raw MAVLink v1/v2 byte stream. Frames with bad CRC or unknown ids are dropped.</summary>
public sealed class MavlinkParser
{
    readonly List<byte> _buf = new(512);
    public long CrcErrors { get; private set; }
    public long FramesParsed { get; private set; }

    public IEnumerable<MavlinkFrame> Push(ReadOnlySpan<byte> data)
    {
        _buf.AddRange(data.ToArray());
        var result = new List<MavlinkFrame>();
        while (true)
        {
            int start = _buf.FindIndex(b => b == 0xFD || b == 0xFE);
            if (start < 0) { _buf.Clear(); break; }
            if (start > 0) _buf.RemoveRange(0, start);
            if (_buf.Count < 3) break;

            bool v2 = _buf[0] == 0xFD;
            int len = _buf[1];
            int header = v2 ? 10 : 6;
            int signature = v2 && (_buf[2] & 0x01) != 0 ? 13 : 0;
            int total = header + len + 2 + signature;
            if (_buf.Count < total) break;

            var frame = _buf.GetRange(0, total).ToArray();
            uint msgId = v2 ? (uint)(frame[7] | frame[8] << 8 | frame[9] << 16) : frame[5];
            if (MavMsg.CrcExtra.TryGetValue(msgId, out var extra))
            {
                ushort crc = MavCrc.Accumulate(frame.AsSpan(1, header - 1 + len), MavCrc.Init);
                crc = MavCrc.Accumulate(extra, crc);
                ushort got = BinaryPrimitives.ReadUInt16LittleEndian(frame.AsSpan(header + len, 2));
                if (crc == got)
                {
                    var payload = frame.AsSpan(header, len).ToArray();
                    result.Add(v2
                        ? new MavlinkFrame(frame[4], frame[5], frame[6], msgId, payload, true)
                        : new MavlinkFrame(frame[2], frame[3], frame[4], msgId, payload, false));
                    FramesParsed++;
                    _buf.RemoveRange(0, total);
                    continue;
                }
                CrcErrors++;
                _buf.RemoveAt(0); // resync
                continue;
            }
            // unknown message: trust the length and skip the whole frame
            _buf.RemoveRange(0, total);
        }
        return result;
    }
}

/// <summary>Builds MAVLink 2 frames.</summary>
public sealed class MavlinkWriter(byte sysId = 255, byte compId = 190)
{
    byte _seq;
    public byte SysId { get; } = sysId;
    public byte CompId { get; } = compId;

    public byte[] Build(uint msgId, ReadOnlySpan<byte> payload)
    {
        int len = payload.Length;
        while (len > 1 && payload[len - 1] == 0) len--; // v2 zero truncation (keep at least one byte)
        var frame = new byte[10 + len + 2];
        frame[0] = 0xFD;
        frame[1] = (byte)len;
        frame[2] = 0;
        frame[3] = 0;
        frame[4] = _seq++;
        frame[5] = SysId;
        frame[6] = CompId;
        frame[7] = (byte)msgId;
        frame[8] = (byte)(msgId >> 8);
        frame[9] = (byte)(msgId >> 16);
        payload[..len].CopyTo(frame.AsSpan(10));
        ushort crc = MavCrc.Accumulate(frame.AsSpan(1, 9 + len), MavCrc.Init);
        crc = MavCrc.Accumulate(MavMsg.CrcExtra[msgId], crc);
        BinaryPrimitives.WriteUInt16LittleEndian(frame.AsSpan(10 + len), crc);
        return frame;
    }
}
