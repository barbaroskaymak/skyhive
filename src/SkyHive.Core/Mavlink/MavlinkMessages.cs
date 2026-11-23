using System.Buffers.Binary;
using System.Text;

namespace SkyHive.Core.Mavlink;

public readonly record struct Heartbeat(uint CustomMode, byte Type, byte Autopilot, byte BaseMode, byte SystemStatus)
{
    public const byte FlagSafetyArmed = 0x80;
    public const byte TypeGcs = 6;
    public const byte AutopilotArdupilot = 3;
    public bool Armed => (BaseMode & FlagSafetyArmed) != 0;
}

public readonly record struct GlobalPositionInt(int LatE7, int LonE7, int AltMm, int RelAltMm, short Vx, short Vy, short Vz, ushort HdgCdeg);
public readonly record struct VfrHud(float Airspeed, float GroundSpeed, float Alt, float Climb, short Heading, ushort Throttle);
public readonly record struct AttitudeMsg(float Roll, float Pitch, float Yaw);
public readonly record struct SysStatus(ushort VoltageMv, short CurrentCa, sbyte BatteryRemaining);
public readonly record struct GpsRawInt(byte FixType, byte Satellites);
public readonly record struct CommandAck(ushort Command, MavResult Result);
public readonly record struct StatusText(byte Severity, string Text);
public readonly record struct HomePositionMsg(int LatE7, int LonE7, int AltMm);

/// <summary>Encoders / decoders for the subset of MAVLink used by SkyHive. Offsets follow MAVLink wire order (fields sorted by size).</summary>
public static class MavCodec
{
    static float F(ReadOnlySpan<byte> s, int o) => BinaryPrimitives.ReadSingleLittleEndian(s[o..]);
    static int I32(ReadOnlySpan<byte> s, int o) => BinaryPrimitives.ReadInt32LittleEndian(s[o..]);
    static uint U32(ReadOnlySpan<byte> s, int o) => BinaryPrimitives.ReadUInt32LittleEndian(s[o..]);
    static short I16(ReadOnlySpan<byte> s, int o) => BinaryPrimitives.ReadInt16LittleEndian(s[o..]);
    static ushort U16(ReadOnlySpan<byte> s, int o) => BinaryPrimitives.ReadUInt16LittleEndian(s[o..]);

    public static Heartbeat DecodeHeartbeat(MavlinkFrame f)
    {
        var b = f.Body(9);
        return new(U32(b, 0), b[4], b[5], b[6], b[7]);
    }

    public static GlobalPositionInt DecodeGlobalPosition(MavlinkFrame f)
    {
        var b = f.Body(28);
        return new(I32(b, 4), I32(b, 8), I32(b, 12), I32(b, 16), I16(b, 20), I16(b, 22), I16(b, 24), U16(b, 26));
    }

    public static VfrHud DecodeVfrHud(MavlinkFrame f)
    {
        var b = f.Body(20);
        return new(F(b, 0), F(b, 4), F(b, 8), F(b, 12), I16(b, 16), U16(b, 18));
    }

    public static AttitudeMsg DecodeAttitude(MavlinkFrame f)
    {
        var b = f.Body(28);
        return new(F(b, 4), F(b, 8), F(b, 12));
    }

    public static SysStatus DecodeSysStatus(MavlinkFrame f)
    {
        var b = f.Body(31);
        return new(U16(b, 14), I16(b, 16), (sbyte)b[30]);
    }

    public static GpsRawInt DecodeGpsRaw(MavlinkFrame f)
    {
        var b = f.Body(30);
        return new(b[28], b[29]);
    }

    public static CommandAck DecodeCommandAck(MavlinkFrame f)
    {
        var b = f.Body(3);
        return new(U16(b, 0), (MavResult)b[2]);
    }

    public static StatusText DecodeStatusText(MavlinkFrame f)
    {
        var b = f.Body(51);
        var text = b.Slice(1, 50);
        int end = text.IndexOf((byte)0);
        return new(b[0], Encoding.ASCII.GetString(end < 0 ? text : text[..end]));
    }

    public static HomePositionMsg DecodeHomePosition(MavlinkFrame f)
    {
        var b = f.Body(52);
        return new(I32(b, 0), I32(b, 4), I32(b, 8));
    }

    public static byte[] EncodeHeartbeat(byte type, byte autopilot, byte baseMode, uint customMode, byte systemStatus)
    {
        var p = new byte[9];
        BinaryPrimitives.WriteUInt32LittleEndian(p, customMode);
        p[4] = type; p[5] = autopilot; p[6] = baseMode; p[7] = systemStatus; p[8] = 3;
        return p;
    }

    public static byte[] EncodeCommandLong(byte targetSys, byte targetComp, ushort command, byte confirmation,
        float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0)
    {
        var p = new byte[33];
        var s = p.AsSpan();
        BinaryPrimitives.WriteSingleLittleEndian(s[0..], p1);
        BinaryPrimitives.WriteSingleLittleEndian(s[4..], p2);
        BinaryPrimitives.WriteSingleLittleEndian(s[8..], p3);
        BinaryPrimitives.WriteSingleLittleEndian(s[12..], p4);
        BinaryPrimitives.WriteSingleLittleEndian(s[16..], p5);
        BinaryPrimitives.WriteSingleLittleEndian(s[20..], p6);
        BinaryPrimitives.WriteSingleLittleEndian(s[24..], p7);
        BinaryPrimitives.WriteUInt16LittleEndian(s[28..], command);
        p[30] = targetSys; p[31] = targetComp; p[32] = confirmation;
        return p;
    }

    /// <summary>Type mask: use position only (ignore velocity, acceleration, yaw and yaw rate).</summary>
    public const ushort PositionOnlyMask = 0x0FF8;
    public const byte FrameGlobalRelativeAltInt = 6;

    public static byte[] EncodeSetPositionTargetGlobalInt(byte targetSys, byte targetComp, uint timeBootMs,
        int latE7, int lonE7, float alt, ushort typeMask = PositionOnlyMask, byte frame = FrameGlobalRelativeAltInt)
    {
        var p = new byte[53];
        var s = p.AsSpan();
        BinaryPrimitives.WriteUInt32LittleEndian(s[0..], timeBootMs);
        BinaryPrimitives.WriteInt32LittleEndian(s[4..], latE7);
        BinaryPrimitives.WriteInt32LittleEndian(s[8..], lonE7);
        BinaryPrimitives.WriteSingleLittleEndian(s[12..], alt);
        BinaryPrimitives.WriteUInt16LittleEndian(s[48..], typeMask);
        p[50] = targetSys; p[51] = targetComp; p[52] = frame;
        return p;
    }

    public static byte[] EncodeRequestDataStream(byte targetSys, byte targetComp, byte streamId, ushort rateHz, bool start)
    {
        var p = new byte[6];
        BinaryPrimitives.WriteUInt16LittleEndian(p, rateHz);
        p[2] = targetSys; p[3] = targetComp; p[4] = streamId; p[5] = (byte)(start ? 1 : 0);
        return p;
    }

    // ---- encoders for telemetry (used by tests and the loopback vehicle) ----
    public static byte[] EncodeGlobalPosition(uint timeMs, int latE7, int lonE7, int altMm, int relAltMm, short vx, short vy, short vz, ushort hdg)
    {
        var p = new byte[28];
        var s = p.AsSpan();
        BinaryPrimitives.WriteUInt32LittleEndian(s[0..], timeMs);
        BinaryPrimitives.WriteInt32LittleEndian(s[4..], latE7);
        BinaryPrimitives.WriteInt32LittleEndian(s[8..], lonE7);
        BinaryPrimitives.WriteInt32LittleEndian(s[12..], altMm);
        BinaryPrimitives.WriteInt32LittleEndian(s[16..], relAltMm);
        BinaryPrimitives.WriteInt16LittleEndian(s[20..], vx);
        BinaryPrimitives.WriteInt16LittleEndian(s[22..], vy);
        BinaryPrimitives.WriteInt16LittleEndian(s[24..], vz);
        BinaryPrimitives.WriteUInt16LittleEndian(s[26..], hdg);
        return p;
    }

    public static byte[] EncodeCommandAck(ushort command, MavResult result)
    {
        var p = new byte[3];
        BinaryPrimitives.WriteUInt16LittleEndian(p, command);
        p[2] = (byte)result;
        return p;
    }
}
