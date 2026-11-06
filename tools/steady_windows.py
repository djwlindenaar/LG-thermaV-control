#!/usr/bin/env python3
"""Calibrate the power target parameters from the Home Assistant recorder.

  steady  30-minute steady running windows: floor eps and UA from the LG Modbus
          pair (with dT offset) and the 1-Wire pair, plus medians.
  circ    pump-only circulation runs: outlet - return per sensor pair, i.e. the
          sensor offset ("Water dT offset" is minus the LG mean).

Reads the recorder database read-only. Entity ids are those of the Daniel
installation; adjust ENT for another one. Standard library only.

  python3 tools/steady_windows.py steady --start 2026-10-27 --end 2026-11-07
  python3 tools/steady_windows.py circ --start 2026-10-27
"""
import argparse
import bisect
import math
import sqlite3
import statistics
from datetime import datetime, timezone

ENT = {
    "lg_out": "sensor.water_aanvoer",
    "lg_in": "sensor.water_retour",
    "d_out": "sensor.heatpump_temp_outflow",
    "d_in": "sensor.heatpump_temp_inflow",
    "flow": "sensor.flow_rate",
    "floor": "sensor.huiskamer_vloer",
    "power": "sensor.lg_therma_total_active_power",
    "comp": "sensor.compressor_speed",
    "defrost": "binary_sensor.defrosting",
    "pump": "binary_sensor.pump_running",
    "outdoor": "sensor.buitentemperatuur",
}
CP = 4168.0  # J/(kg K), as in the firmware
STEP = 30  # s, resampling interval
WIN = 30 * 60 // STEP  # samples per steady window


def ts(s):
    return datetime.fromisoformat(s).replace(tzinfo=timezone.utc).timestamp()


def load(con, start, end):
    series = {}
    for k, e in ENT.items():
        rows = con.execute(
            "select s.last_updated_ts, s.state from states s join states_meta m on s.metadata_id=m.metadata_id "
            "where m.entity_id=? and s.last_updated_ts between ? and ? order by s.last_updated_ts",
            (e, start - 3600, end)).fetchall()
        t, v = [], []
        for r_t, r_v in rows:
            if r_v in ("on", "off"):
                x = 1.0 if r_v == "on" else 0.0
            else:
                try:
                    x = float(r_v)
                except (TypeError, ValueError):
                    x = math.nan
            t.append(r_t)
            v.append(x)
        series[k] = (t, v)
    return series


def at(series, k, t):
    tt, vv = series[k]
    i = bisect.bisect_right(tt, t) - 1
    return vv[i] if i >= 0 else math.nan


def mean(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return sum(xs) / len(xs) if xs else math.nan


def sd(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return statistics.pstdev(xs) if xs else math.nan


def median(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return statistics.median(xs) if xs else math.nan


def steady(series, start, end, offset):
    samples = []
    t = start
    while t < end:
        samples.append((t, {k: at(series, k, t) for k in ENT}))
        t += STEP
    print("window(UTC)   P[W] comp flow outd floor | LG+off dT   eps    UA |  1W dT   eps    UA")
    res = {"lg": [], "d": [], "d_bad": 0, "n": 0}
    i = 0
    while i + WIN <= len(samples):
        seg = samples[i:i + WIN]
        g = lambda k: [x[1][k] for x in seg]
        comp = g("comp")
        ok = (min(comp) > 15 and sd(comp) < 3 and max(g("defrost")) == 0 and min(g("pump")) == 1
              and min(g("flow")) > 5 and sd(g("power")) < 150)
        if not ok:
            i += WIN // 3
            continue
        res["n"] += 1
        mcp = mean(g("flow")) / 60 * CP
        fl, p = mean(g("floor")), mean(g("power"))
        cols = []
        for key, a, b, off in (("lg", "lg_out", "lg_in", offset), ("d", "d_out", "d_in", 0.0)):
            to, ti = mean(g(a)), mean(g(b))
            dt = to - ti + off
            eps = dt / (to - fl) if to - fl > 0.1 else math.nan
            ua = -mcp * math.log(1 - eps) if 0 < eps < 1 else math.nan
            if key == "d" and not eps < 1:
                res["d_bad"] += 1
            res[key].append(ua)
            cols.append(f"{dt:5.2f} {eps:5.2f} {ua:5.0f}")
        print(f"{datetime.fromtimestamp(seg[0][0], timezone.utc):%m-%d %H:%M}  {p:5.0f} {mean(comp):4.0f} "
              f"{mean(g('flow')):4.1f} {mean(g('outdoor')):4.1f} {fl:5.2f} | " + " | ".join(cols))
        i += WIN

    def iqr(xs):
        xs = sorted(x for x in xs if not math.isnan(x))
        if len(xs) < 4:
            return "n/a"
        q = statistics.quantiles(xs, n=4)
        return f"{q[0]:.0f}-{q[2]:.0f}"

    print(f"\n{res['n']} windows")
    print(f"UA LG+{offset:.2f}: median {median(res['lg']):.0f} W/K, IQR {iqr(res['lg'])}")
    print(f"UA 1-Wire: median {median(res['d']):.0f} W/K (valid windows), IQR {iqr(res['d'])}, "
          f"eps >= 1 in {res['d_bad']} windows")


def circ(series):
    # sample 10 s before the pump stops after a pump-only run; earlier samples still carry heat from the unit
    t_p, v_p = series["pump"]
    rows = []
    print("pump-only run end (UTC)  dur   LGo-LGi  1Wo-1Wi  LGo-1Wo  LGi-1Wi")
    for i in range(len(t_p) - 1):
        if v_p[i] == 1 and v_p[i + 1] == 0:
            t0, t1 = t_p[i], t_p[i + 1]
            if at(series, "comp", t1 - 5) != 0 or at(series, "comp", t0) != 0:
                continue
            t = t1 - 10
            lo, li, do, di = (at(series, k, t) for k in ("lg_out", "lg_in", "d_out", "d_in"))
            rows.append((lo - li, do - di, lo - do, li - di))
            print(f"{datetime.fromtimestamp(t1, timezone.utc):%m-%d %H:%M}  {t1 - t0:5.0f}  "
                  f"{lo - li:7.2f}  {do - di:7.2f}  {lo - do:7.2f}  {li - di:7.2f}")
    for j, name in enumerate(("LGo-LGi", "1Wo-1Wi", "LGo-1Wo", "LGi-1Wi")):
        x = [r[j] for r in rows if not math.isnan(r[j])]
        if x:
            print(f"{name}: mean {statistics.mean(x):.2f} sd {statistics.pstdev(x):.2f} n {len(x)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("steady", "circ"))
    ap.add_argument("--db", default="/config/home-assistant_v2.db")
    ap.add_argument("--start", required=True, help="UTC date/time, ISO format")
    ap.add_argument("--end", default=None, help="UTC date/time, ISO format (default: now)")
    ap.add_argument("--offset", type=float, default=0.39, help="LG dT offset in K (steady mode)")
    a = ap.parse_args()
    start = ts(a.start)
    end = ts(a.end) if a.end else datetime.now(timezone.utc).timestamp()
    con = sqlite3.connect(f"file:{a.db}?mode=ro", uri=True)
    series = load(con, start, end)
    if a.mode == "steady":
        steady(series, start, end, a.offset)
    else:
        circ(series)


if __name__ == "__main__":
    main()
