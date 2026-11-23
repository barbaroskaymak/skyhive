using System.Globalization;
using System.Text;
using SkyHive.Core.Geo;
using SkyHive.Core.Swarm;

namespace SkyHive.Core.Commands;

public sealed record CommandResult(bool Ok, string Message);

/// <summary>
/// Text command console (Turkish and English keywords). Examples:
/// "kalk 25", "formasyon v 12", "tara 300x200", "yörünge d2-d4 40", "git d1 120 -40 30", "takip t3", "eve dön".
/// </summary>
public sealed class CommandInterpreter(SwarmSession session)
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static readonly string Help = string.Join('\n',
        "arm / disarm [seçim]            — motorları arm/disarm et",
        "kalk [seçim] [irtifa]          — GUIDED + arm + kalkış (vars. 20 m)",
        "in [seçim]  ·  eve dön [seçim] — LAND / RTL",
        "dur [seçim]                    — bulunduğu yerde askıda bekle",
        "git <dN> <kuzey> <doğu> [irt]  — eve göre metre; ya da git dN @lat,lon[,alt]",
        "hız [seçim] <m/s>              — seyir hızı",
        "formasyon <hat|kol|v|çember|ızgara> [aralık] [lider dN]",
        "yörünge [seçim] [yarıçap] [kuzey doğu] — sürü merkezinde/noktada çember",
        "tara [seçim] [GxY] [kuzey doğu] — alanı dronlara bölüp şerit tarama",
        "devriye [seçim] [kenar]        — kare devriye (döngü)",
        "takip <Tn> [seçim]             — tespit edilen hedefin etrafında yörünge",
        "hedefler · demo · yardım",
        "seçim: hepsi | d1 | d1,d3 | d2-d4");

    static string Normalize(string s)
    {
        var sb = new StringBuilder(s.Length);
        foreach (var ch in s.Trim().ToLower(new CultureInfo("tr-TR")))
            sb.Append(ch switch { 'ı' => 'i', 'ş' => 's', 'ğ' => 'g', 'ü' => 'u', 'ö' => 'o', 'ç' => 'c', 'â' => 'a', _ => ch });
        return sb.ToString();
    }

    List<int>? TryParseSelection(string tok)
    {
        if (tok is "hepsi" or "all" or "tum" or "tumu" or "*" or "surü" or "suru") return session.Agents.Select(a => a.Id).ToList();
        var ids = new List<int>();
        foreach (var part in tok.Split(','))
        {
            var p = part.Trim();
            if (p.Length == 0) continue;
            var range = p.Split('-');
            if (range.Length == 2 && TryDrone(range[0], out var a) && TryDrone(range[1], out var b))
            {
                for (int i = Math.Min(a, b); i <= Math.Max(a, b); i++) ids.Add(i);
            }
            else if (TryDrone(p, out var one)) ids.Add(one);
            else return null;
        }
        ids = ids.Where(id => session.Agents.Any(x => x.Id == id)).Distinct().ToList();
        return ids.Count > 0 ? ids : null;
    }

    static bool TryDrone(string s, out int id)
    {
        id = 0;
        return s.StartsWith('d') && int.TryParse(s[1..], out id);
    }

    static bool TryNum(string s, out double v) => double.TryParse(s.Replace(',', '.'), NumberStyles.Float, Inv, out v);

    public async Task<CommandResult> ExecuteAsync(string input, int? selectedDrone = null)
    {
        if (string.IsNullOrWhiteSpace(input)) return new(false, "Boş komut");
        var tokens = Normalize(input).Split(' ', StringSplitOptions.RemoveEmptyEntries).ToList();
        var verb = tokens[0];
        tokens.RemoveAt(0);
        if (verb == "eve" && tokens.FirstOrDefault() is "don" or "dön") tokens.RemoveAt(0);

        List<int>? sel = null;
        if (tokens.Count > 0 && TryParseSelection(tokens[0]) is { } s) { sel = s; tokens.RemoveAt(0); }
        var nums = tokens.Select(t => TryNum(t, out var v) ? v : (double?)null).Where(v => v.HasValue).Select(v => v!.Value).ToList();
        string Who(IReadOnlyCollection<int>? ids) => ids is null || ids.Count == session.Agents.Count ? "tüm sürü" : string.Join(",", ids.Select(i => $"D{i}"));
        var c = session.Controller;

        try
        {
            switch (verb)
            {
                case "yardim" or "help" or "?":
                    return new(true, Help);

                case "arm":
                    return new(true, $"{await c.ArmAsync(sel, true)} drone arm edildi");
                case "disarm":
                    return new(true, $"{await c.ArmAsync(sel, false)} drone disarm edildi");

                case "kalk" or "takeoff" or "kalkis":
                {
                    var alt = nums.Count > 0 ? nums[0] : 20;
                    if (alt is < 2 or > 120) return new(false, "İrtifa 2–120 m olmalı");
                    var ok = await c.TakeoffAsync(sel, alt);
                    return new(ok > 0, $"{Who(sel)}: {ok} drone {alt:F0} m'ye kalkıyor");
                }
                case "in" or "land" or "inis":
                    await c.LandAsync(sel);
                    return new(true, $"{Who(sel)}: LAND");
                case "eve" or "rtl" or "don":
                    await c.RtlAsync(sel);
                    return new(true, $"{Who(sel)}: eve dönüş (RTL)");
                case "dur" or "hold" or "bekle" or "stop":
                    c.Hold(sel);
                    return new(true, $"{Who(sel)}: askıda bekliyor");

                case "hiz" or "speed":
                    if (nums.Count == 0) return new(false, "Hız değeri gerekli: hız 8");
                    await c.SetSpeedAsync(sel, nums[0]);
                    return new(true, $"{Who(sel)}: hız {nums[0]:F1} m/s");

                case "git" or "goto":
                {
                    var id = sel is { Count: 1 } ? sel[0] : selectedDrone ?? session.Agents[0].Id;
                    var at = tokens.FirstOrDefault(t => t.StartsWith('@'));
                    if (at is not null)
                    {
                        var parts = at[1..].Split(',').Select(p => double.Parse(p, Inv)).ToArray();
                        var geo = new GeoPoint(parts[0], parts[1], parts.Length > 2 ? parts[2] : 20);
                        c.Goto(id, GeoMath.ToLocal(session.Origin, geo), geo.Alt);
                        return new(true, $"D{id} → {geo}");
                    }
                    if (nums.Count < 2) return new(false, "Kullanım: git d1 <kuzey> <doğu> [irtifa]");
                    c.Goto(id, new Vec2(nums[0], nums[1]), nums.Count > 2 ? nums[2] : null);
                    return new(true, $"D{id} → N{nums[0]:F0} E{nums[1]:F0}");
                }

                case "formasyon" or "formation" or "dizil":
                {
                    var typeTok = tokens.FirstOrDefault(t => !TryNum(t, out _)) ?? "v";
                    FormationType? type = typeTok switch
                    {
                        "hat" or "line" or "yan" => FormationType.Line,
                        "kol" or "column" or "sira" => FormationType.Column,
                        "v" or "kama" or "wedge" => FormationType.Wedge,
                        "cember" or "circle" or "halka" => FormationType.Circle,
                        "izgara" or "grid" or "kare" => FormationType.Grid,
                        _ => null,
                    };
                    if (type is null) return new(false, "Formasyon: hat | kol | v | çember | ızgara");
                    var spacing = nums.Count > 0 ? nums[0] : 10;
                    var leaderTok = tokens.SkipWhile(t => t is not ("lider" or "leader")).Skip(1).FirstOrDefault();
                    var leader = leaderTok is not null && TryDrone(leaderTok, out var lid) ? lid : (sel?.FirstOrDefault() ?? session.Agents[0].Id);
                    var n = c.Formation(type.Value, spacing, leader, sel);
                    return new(true, $"{Formations.DisplayName(type.Value)} formasyonu: lider D{leader}, {n} takipçi, aralık {spacing:F0} m");
                }

                case "yorunge" or "orbit" or "daire":
                {
                    var r = nums.Count > 0 ? nums[0] : 40;
                    var center = nums.Count >= 3 ? new Vec2(nums[1], nums[2]) : c.Centroid(sel);
                    var alt = session.Agents.Where(a => sel?.Contains(a.Id) ?? true).Select(a => a.Alt).DefaultIfEmpty(20).Max();
                    var n = c.Orbit(sel, center, r, Math.Max(alt, 15));
                    return new(true, $"{n} drone {center} etrafında r={r:F0} m yörüngede");
                }

                case "tara" or "search" or "tarama":
                {
                    double w = 240, h = 180;
                    var dim = tokens.FirstOrDefault(t => t.Contains('x'));
                    if (dim is not null && dim.Split('x') is [var a, var b] && TryNum(a, out var wa) && TryNum(b, out var hb)) { w = wa; h = hb; }
                    else if (nums.Count >= 2) { w = nums[0]; h = nums[1]; nums = nums.Skip(2).ToList(); }
                    var rest = dim is not null ? nums : nums;
                    var center = rest.Count >= 2 ? new Vec2(rest[^2], rest[^1]) : c.Centroid(sel);
                    var alt = Math.Max(18, session.Agents.Where(a => sel?.Contains(a.Id) ?? true).Select(a => a.Alt).DefaultIfEmpty(20).Max());
                    var len = c.Search(sel, center, w, h, alt, session.Vision.Camera);
                    var cnt = sel?.Count ?? session.Agents.Count;
                    return new(true, $"{w:F0}×{h:F0} m alan {cnt} şeride bölündü, toplam rota {len:F0} m (şerit aralığı {session.Vision.Camera.FootprintWidth(alt) * 0.75:F0} m)");
                }

                case "devriye" or "patrol":
                {
                    var size = nums.Count > 0 ? nums[0] : 120;
                    var alt = Math.Max(15, session.Agents.Select(a => a.Alt).DefaultIfEmpty(20).Max());
                    var n = c.Patrol(sel, c.Centroid(sel), size, alt);
                    return new(true, $"{n} drone {size:F0} m kare devriyede");
                }

                case "takip" or "track":
                {
                    var tTok = tokens.FirstOrDefault(t => t.StartsWith('t') && int.TryParse(t[1..], out _));
                    int tid;
                    if (tTok is not null) tid = int.Parse(tTok[1..]);
                    else if (nums.Count > 0) tid = (int)nums[0];
                    else
                    {
                        var first = session.Tracker.Snapshot().Where(t => t.Confirmed).OrderByDescending(t => t.LastSeen).FirstOrDefault();
                        if (first is null) return new(false, "Henüz doğrulanmış hedef yok — önce 'tara'");
                        tid = first.Id;
                    }
                    var d = c.Track(tid, sel);
                    return d is null ? new(false, "Havada uygun drone yok") : new(true, $"{d.Name} → T{tid} takibinde");
                }

                case "hedefler" or "targets":
                {
                    var list = session.Tracker.Snapshot();
                    if (list.Count == 0) return new(true, "Tespit yok");
                    return new(true, string.Join('\n', list.Select(t => $"{t.Label}  {t.Position}  {(t.Confirmed ? "✔" : "?")}  {t.Hits} görüş  {t.Velocity.Length:F1} m/s")));
                }

                case "demo" or "senaryo":
                    _ = Task.Run(() => session.RunDemoAsync());
                    return new(true, "Demo senaryosu başlatıldı");

                default:
                    return new(false, $"Bilinmeyen komut: {verb} — 'yardım' yazın");
            }
        }
        catch (Exception ex)
        {
            return new(false, ex.Message);
        }
    }
}
