using SkyHive.Core.Mavlink;

namespace SkyHive.Core.Tests;

/// <summary>Reference frames generated with pymavlink (ardupilotmega dialect) — proves wire compatibility with ArduPilot.</summary>
public class MavlinkTests
{
    static byte[] Hex(string s) => Convert.FromHexString(s);

    const string Heartbeat = "fd090000000101000000040000000203890403dbb3";
    const string GlobalPos = "fd1c000007010121000040e2010003f66e1808fa5b111ec8000014500000960024ffddffaa6927f8";
    const string Ack = "fd0200000801014d000090016a27";
    const string Status = "fd180000090101fd00000450726541726d3a20475053206e6f74206865616c7468794f87";
    const string HeartbeatV1 = "fe0900010100050000000203810403beb9";

    [Fact]
    public void Parses_heartbeat_v2()
    {
        var f = Assert.Single(new MavlinkParser().Push(Hex(Heartbeat)));
        var hb = MavCodec.DecodeHeartbeat(f);
        Assert.Equal(1, f.SysId);
        Assert.Equal(4u, hb.CustomMode); // GUIDED
        Assert.True(hb.Armed);
        Assert.Equal(SkyHive.Core.Mavlink.Heartbeat.AutopilotArdupilot, hb.Autopilot);
    }

    [Fact]
    public void Parses_heartbeat_v1()
    {
        var f = Assert.Single(new MavlinkParser().Push(Hex(HeartbeatV1)));
        Assert.False(f.IsV2);
        Assert.Equal(5u, MavCodec.DecodeHeartbeat(f).CustomMode); // LOITER
    }

    [Fact]
    public void Parses_global_position()
    {
        var f = Assert.Single(new MavlinkParser().Push(Hex(GlobalPos)));
        var p = MavCodec.DecodeGlobalPosition(f);
        Assert.Equal(409925123, p.LatE7);
        Assert.Equal(291240456, p.LonE7);
        Assert.Equal(20500, p.RelAltMm);
        Assert.Equal(-220, p.Vy);
        Assert.Equal(27050, p.HdgCdeg);
    }

    [Fact]
    public void Parses_truncated_command_ack_and_statustext()
    {
        var frames = new MavlinkParser().Push(Hex(Ack + Status)).ToList();
        Assert.Equal(2, frames.Count);
        var ack = MavCodec.DecodeCommandAck(frames[0]);
        Assert.Equal(MavCmd.ComponentArmDisarm, ack.Command);
        Assert.Equal(MavResult.Accepted, ack.Result);
        Assert.Equal("PreArm: GPS not healthy", MavCodec.DecodeStatusText(frames[1]).Text);
    }

    [Fact]
    public void Survives_garbage_split_chunks_and_corruption()
    {
        var parser = new MavlinkParser();
        var bad = Hex(Heartbeat);
        bad[12] ^= 0xFF; // corrupt payload → CRC failure
        var stream = new byte[] { 0x00, 0x13, 0x37 }.Concat(bad).Concat(Hex(GlobalPos)).Concat(Hex(Heartbeat)).ToArray();
        var frames = new List<MavlinkFrame>();
        for (int i = 0; i < stream.Length; i += 5)
            frames.AddRange(parser.Push(stream.AsSpan(i, Math.Min(5, stream.Length - i))));
        Assert.Equal(new uint[] { MavMsg.GlobalPositionInt, MavMsg.Heartbeat }, frames.Select(f => f.MsgId));
        Assert.True(parser.CrcErrors >= 1);
    }

    [Fact]
    public void Encodes_command_long_identical_to_pymavlink()
    {
        var w = new MavlinkWriter(255, 190);
        var bytes = w.Build(MavMsg.CommandLong, MavCodec.EncodeCommandLong(1, 1, MavCmd.ComponentArmDisarm, 0, 1));
        Assert.Equal("fd20000000ffbe4c00000000803f000000000000000000000000000000000000000000000000900101019e4e", Convert.ToHexString(bytes).ToLowerInvariant());
    }

    [Fact]
    public void Encodes_position_target_and_data_stream_identical_to_pymavlink()
    {
        var w = new MavlinkWriter(255, 190);
        w.Build(MavMsg.Heartbeat, new byte[9]); // seq 0
        var spt = w.Build(MavMsg.SetPositionTargetGlobalInt, MavCodec.EncodeSetPositionTargetGlobalInt(1, 1, 5000, 409930000, 291250000, 25.5f));
        var rds = w.Build(MavMsg.RequestDataStream, MavCodec.EncodeRequestDataStream(1, 1, 0, 4, true));
        Assert.Equal("fd35000001ffbe5600008813000010096f18501f5c110000cc410000000000000000000000000000000000000000000000000000000000000000f80f0101069a6d", Convert.ToHexString(spt).ToLowerInvariant());
        Assert.Equal("fd06000002ffbe42000004000101000123da", Convert.ToHexString(rds).ToLowerInvariant());
    }
}
