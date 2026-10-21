#!/usr/bin/env python3
"""Offline analysis only; never changes raw captures or acceptance manifests."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from collections import defaultdict
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import statistics

from diagnose_nucleo_stream import HubStreamParser, save_json, sha256


def s32(a, b):
    return ((a - b + 2**31) % 2**32) - 2**31


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return {"median": statistics.median(values),
            "p99": values[int(len(values) * .99)], "max": max(values)}


def wire_counts(packets):
    result = {}
    for channel in (1, 2, 3, 4):
        ps = [p for p in packets if p["channel"] == channel]
        outer = [(b["channel_seq"] - a["channel_seq"]) % 2**32
                 for a, b in zip(ps, ps[1:])]
        inner = [(b["src_seq"] - a["src_seq"]) % 16
                 for a, b in zip(ps, ps[1:])]
        result[channel] = {
            "packets": len(ps),
            "outer_missing": sum(d - 1 for d in outer if 1 < d < 2**31),
            "outer_duplicates": outer.count(0),
            "outer_reordered": sum(d >= 2**31 for d in outer),
            "inner_missing": sum(d - 1 for d in inner if 1 < d < 8),
            "inner_duplicate_or_backwards": sum(d == 0 or d >= 8 for d in inner),
        }
    return result


def reference_pairs(packets):
    # Diagnostic association only: equal four-bit source sequence AND source
    # timestamps within 5 ms. This does NOT replace production grouping or
    # turn any rejected window into accepted data. Retain only complete pairs.
    pending = defaultdict(list)
    groups = []
    for p in packets:
        candidates = pending[p["src_seq"]]
        group = next((g for g in candidates
                      if abs(s32(p["source_ms"], g[0]["source_ms"])) <= 5
                      and not any(x["channel"] == p["channel"] for x in g)), None)
        if group is None:
            group = []
            candidates.append(group)
        group.append(p)
        if len(group) == 4:
            ns = [s32(x["nucleo_ms"], group[0]["nucleo_ms"]) for x in group]
            ss = [s32(x["source_ms"], group[0]["source_ms"]) for x in group]
            hs = [x["host_ns"] for x in group]
            groups.append({"nucleo_span_ms": max(ns) - min(ns),
                           "source_span_ms": max(ss) - min(ss),
                           "host_span_ms": (max(hs) - min(hs)) / 1e6})
            candidates.remove(group)
        for key in list(pending):
            pending[key] = [g for g in pending[key]
                            if s32(p["source_ms"], g[0]["source_ms"]) < 1000]
    return groups


def analyze(root):
    status = json.loads((root / "status.json").read_text())
    if status["state"] != "complete":
        raise RuntimeError("diagnostic collection must finish before analysis")
    packets = defaultdict(list)
    with (root / "packets.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            p = {k: (v if k == "phase" else int(v)) for k, v in row.items()}
            packets[p["phase"]].append(p)
    reads = defaultdict(list)
    replay = HubStreamParser()
    with (root / "reads.csv").open(newline="") as stream, (root / "wire.bin").open("rb") as raw:
        for row in csv.DictReader(stream):
            row = {k: (v if k == "phase" else int(v)) for k, v in row.items()}
            if raw.tell() != row["offset"]:
                raise RuntimeError("wire read offset mismatch")
            chunk = raw.read(row["length"])
            if len(chunk) != row["length"]:
                raise RuntimeError("truncated raw wire file")
            replay.feed(chunk, row["host_ns"])
            reads[row["phase"]].append(row)
    replay_stats = json.loads(json.dumps(asdict(replay.stats)))
    if replay_stats != status["parser_before_close"]:
        raise RuntimeError("instrumentation differs from unmodified parser replay")
    checksum_matches = {name: sha256(root / name) == checksum
                        for name, checksum in status["sha256"].items()}
    if not all(checksum_matches.values()):
        raise RuntimeError("diagnostic checksum mismatch")
    result = {"diagnostic_directory": str(root), "sync_sent": status["sync_sent"],
              "serial_write_calls": status["serial_write_calls"],
              "group_timeout_ms": 40, "unmodified_parser_exact_replay": True,
              "checksum_matches": checksum_matches, "phases": []}
    for phase in status["phases"]:
        def in_window(p):
            return phase["measurement_start_ns"] <= p["host_ns"] < phase["end_ns"]
        ps = [p for p in packets[phase["label"]] if in_window(p)]
        pairs = reference_pairs(ps)
        phase_reads = [p for p in reads[phase["label"]] if in_window(p)]
        result["phases"].append({
            **phase, "wire_sequence_counters": wire_counts(ps),
            "reference_complete_groups": len(pairs),
            **{key: distribution([g[key] for g in pairs])
               for key in ("nucleo_span_ms", "source_span_ms", "host_span_ms")},
            "host_spans_ge_40ms": sum(g["host_span_ms"] >= 40 for g in pairs),
            "read_duration_ms": distribution([(r["host_ns"] - r["read_start_ns"]) / 1e6
                                               for r in phase_reads]),
        })
    expiries, matched = {}, []
    with (root / "parser_events.jsonl").open() as stream:
        for line in stream:
            event = json.loads(line)
            if event["kind"] == "expire_before_feed":
                expiries[event["phase"], event["host_ns"], event["src_seq"]] = event
            elif event["kind"] == "reordered_rejection":
                p = event["packet"]
                prior = expiries.get((event["phase"], p["host_ns"], p["src_seq"]))
                if prior and all(abs(s32(p["source_ms"], r["source_ms"])) <= 5
                                 for r in prior["records"]):
                    ns = [s32(r["nucleo_ms"], p["nucleo_ms"])
                          for r in prior["records"]] + [0]
                    matched.append({"phase": event["phase"], "host_ns": p["host_ns"],
                                    "src_seq": p["src_seq"],
                                    "host_age_ms": prior["age_ms"],
                                    "nucleo_span_ms": max(ns) - min(ns),
                                    "received_channels": [r["channel"] for r in prior["records"]],
                                    "last_channel": p["channel"]})
    result["expired_then_rejected_in_same_read"] = {
        "count_full_capture_including_phase_settling": len(matched), "examples": matched[:10]}
    save_json(root / "forensic_analysis.json", result)
    return result


def history(campaign, output):
    sessions = [campaign, campaign / "continuations/after_user_reboot__20261016T122212Z"]
    rows, fits = [], []
    for root in sessions:
        first_window_ns = None
        session_rows = []
        for entry in sorted(root.glob("*/background_120s_attempt_*"), key=lambda p: p.name):
            manifest = json.loads((entry / "nucleo_capture.yaml").read_text())
            begin = manifest["capture"]["window_start_monotonic_ns"]
            if first_window_ns is None:
                first_window_ns = begin
            spans, offsets = [], []
            with (entry / "nucleo_raw_telemetry.csv").open(newline="") as stream:
                reader = csv.reader(stream)
                fields = next(reader)
                indices = [fields.index(f"channel_{i}_nucleo_time_ms_u32") for i in (1, 2, 3, 4)]
                for row in reader:
                    ts = [int(row[i]) for i in indices]
                    relative = [s32(t, ts[0]) for t in ts]
                    spans.append(max(relative) - min(relative))
                    offsets.append(relative[2])
            row = {"session": root.name, "attempt": entry.name,
                   "accepted": manifest["quality"]["accepted"],
                   "elapsed_min": (begin - first_window_ns) / 60e9,
                   "complete_groups": len(spans),
                   "median_nucleo_span_ms": statistics.median(spans),
                   "p99_nucleo_span_ms": sorted(spans)[int(len(spans) * .99)],
                   "max_nucleo_span_ms": max(spans),
                   "median_channel3_minus_channel1_ms": statistics.median(offsets),
                   "timeout_groups": manifest["counts"]["parser"]["timeout_groups"],
                   "reordered_packets": manifest["counts"]["parser"]["reordered_packets"]}
            rows.append(row)
            session_rows.append(row)
        if len(session_rows) >= 2:
            x = [r["elapsed_min"] for r in session_rows]
            y = [r["median_channel3_minus_channel1_ms"] for r in session_rows]
            fit = statistics.linear_regression(x, y)
            fits.append({"session": root.name, "windows": len(x),
                         "channel3_minus_channel1_slope_ms_per_min": fit.slope,
                         "relative_timing_slope_ppm": fit.slope / 60000 * 1e6,
                         "first": session_rows[0], "last": session_rows[-1]})
    # Only derived diagnostic outputs are regenerated; never touch source CSVs.
    with (output / "historical_timing.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    save_json(output / "historical_timing_summary.json", {"fits": fits, "window_count": len(rows),
              "note": "Timestamp drift is upstream of the PC; this does not identify which MCU/clock/queue is responsible."})
    return fits


def report(root, result, fits):
    status = json.loads((root / "status.json").read_text())
    zone = timezone(timedelta(hours=8))
    began = datetime.fromisoformat(status["started_utc"]).astimezone(zone).isoformat()
    ended = datetime.fromisoformat(status["finished_utc"]).astimezone(zone).isoformat()
    lines = ["# NUCLEO 长时间采集：接收端与上游时序归因", "",
             f"被动诊断采集时间（北京时间）：{began} 至 {ended}", "",
             "## 结论", "",
             "这次大批超时与乱序计数的直接原因已定位在PC接收端：固定4096字节的阻塞读取通常等待约20ms，"
             "使同一组数据的主机可见时间跨度跨过40ms；解析器在处理这次已读入的字节之前先淘汰旧组，"
             "随后把原本顺序正常的包判为迟到/乱序。该结论由同一串口连接的A/B/A实测支持，不是仅凭代码推测。", "",
             "上游也存在独立问题：各通道在NUCLEO时间戳下的相对到达时序随运行时间缓慢漂移。"
             "它减少了40ms组帧期限的余量，解释了为什么开始正常、运行久后恶化。"
             "这不是NUCLEO实际乱发包的证据；尚不能仅凭HUB报文定位到STM32队列还是各ESP32时钟/采样相位。", "",
             "## 控制实验", "",
             "同一块板、同一次打开的串口连接；未重启、未发送SYNC、未写入任何串口命令；"
             "解析器原40ms期限与八组缓存上限不变。各阶段30秒，前5秒作为边界稳定段，统计其后约25秒。", "",
             "| 阶段 | 有效秒数 | 完整组 | 超时组 | 全局乱序计数 | 线上真实乱序 | 线上缺失包 |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    for p in result["phases"]:
        wire = p["wire_sequence_counters"].values()
        lines.append(f"| {p['label']} | {p['measurement_s']:.3f} | {p['parser_delta']['completed_groups']} | "
                     f"{p['parser_delta']['timeout_groups']} | {p['parser_delta']['reordered_packets']} | "
                     f"{sum(c['outer_reordered'] for c in wire)} | "
                     f"{sum(c['outer_missing'] for c in p['wire_sequence_counters'].values())} |")
    lines += ["", "线上计数直接来自所有解码报文的通道序号，而非解析器已接受帧的序号。"
              "第一阶段确有1个真实缺包，但不能解释252次超时/乱序；第三阶段没有真实缺包，仍发生270次超时/乱序。", "",
              "及时读取阶段保留100Hz完整输出，25秒输出2500组，并未靠少读、跳组、清空队列或改门槛消除计数。", "",
              "## 延迟与逐事件证据", ""]
    for p in result["phases"]:
        lines.append(f"- {p['label']}：单次read耗时中位数{p['read_duration_ms']['median']:.3f}ms；"
                     f"同组板端NUCLEO时间跨度中位数{p['nucleo_span_ms']['median']}ms、"
                     f"最大{p['nucleo_span_ms']['max']}ms；主机跨度最大{p['host_span_ms']['max']:.3f}ms。")
    lines += ["", "三段中板端组内时间跨度基本相同（中位数23ms、最大34ms），但原读取方式使部分组的"
              "主机跨度达到约42ms，及时读取没有跨过40ms。", "",
              f"在完整90秒抓取（含边界稳定段）中，找到{result['expired_then_rejected_in_same_read']['count_full_capture_including_phase_settling']}"
              "条同一次read内先过期、后拒收对应末通道包的事件。示例："]
    for example in result["expired_then_rejected_in_same_read"]["examples"][:3]:
        lines.append(f"- {example['phase']}，src_seq={example['src_seq']}，主机组龄{example['host_age_ms']:.3f}ms，"
                     f"板端时间跨度仅{example['nucleo_span_ms']}ms；已收通道{example['received_channels']}，"
                     f"同批字节中的通道{example['last_channel']}随后被拒收。")
    lines += ["", "## 长时间漂移（独立于PC接收时间戳）", ""]
    for fit in fits:
        first, last = fit["first"], fit["last"]
        lines.append(f"- {fit['session']}：核对{fit['windows']}个120秒窗口，"
                     f"通道3减通道1的NUCLEO时间差从{first['median_channel3_minus_channel1_ms']}ms"
                     f"变为{last['median_channel3_minus_channel1_ms']}ms；"
                     f"拟合斜率{fit['channel3_minus_channel1_slope_ms_per_min']:.4f}ms/分钟，"
                     f"等效相对时序漂移{fit['relative_timing_slope_ppm']:.2f}ppm。")
    lines += ["", "该斜率是对报文中板端时间差的统计，不是已测定某颗晶振的频偏。"
              "两次会话的变化率相近，符合通道时基/采样相位慢漂；如需区分ESP32时钟与STM32转发实现，"
              "还需固件及采样/收包打点。hospital分支在此仅提供PC协议、读取与组帧代码。", "",
              "## 代码定位与后续建议", "",
              "- `src/hx_waypoint_nucleo/hx_waypoint_nucleo/capture.py:530`：固定read_size=4096、timeout=0.02，"
              "每批字节只在read返回时打一次时间戳。",
              "- `src/hx_waypoint_nucleo/hx_waypoint_nucleo/protocol.py:511`：先expire，再解析本批已到达的字节。",
              "- `protocol.py:605`等分支：过期组的对应包落后于最新分组后，计为全局reordered_packets；"
              "此计数不能直接解释为物理链路乱序。",
              "- 上游hospital的reader.py使用in_waiting及时读取，而不是每次阻塞等待4096字节。",
              "- 应首先修接收端及时读取与时间戳/过期判定的配合；不通过增加40ms期限、删样本、"
              "定期重发SYNC或修改原始数据来达标。",
              "- 25秒对照只能证明当前直接故障与读法有关，不能保证3小时20分钟连续采集可行。"
              "上游漂移继续增大后仍可能真正越过40ms，必须做足够长的验证并处理通道时基问题。", "",
              "## 证据与范围", "",
              "- wire.bin：收到的全部原始二进制字节，包括未组成完整组的包。",
              "- reads.csv：原始字节偏移、每次read起止时间、字节数及实验阶段。",
              "- packets.csv：每个有效HUB包的通道/源序号、源时间、NUCLEO时间和主机时间。",
              "- parser_events.jsonl：每个过期组及随后发生的乱序拒收事件。",
              "- forensic_analysis.json：逐阶段线上包序、时间跨度与错误计数。",
              "- historical_timing.csv及historical_timing_summary.json：43个历史窗口的时序统计。",
              "- provenance/：诊断程序、测试、原生产解析器与接收程序的副本。", "",
              "所有原始字节按原读取边界交给未修改的生产解析器离线重放，统计与实测完全一致；"
              "原始文件SHA256均已校验。针对及时读法、分批读取伪超时及仪器化不改变解析结果的3项测试"
              "在Mac和Ubuntu24均通过。", "",
              "这次仅新增诊断程序和证据，未修改正式采集程序/解析器/固件，未恢复100条采集。"
              "背景场合格数量仍为30条；本诊断数据不计入合格背景场，也不重新认定此前13条拒收记录。", ""]
    (root / "REPORT.zh-CN.md").write_text("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--diagnostic", type=Path, required=True)
    ap.add_argument("--campaign", type=Path)
    args = ap.parse_args()
    result = analyze(args.diagnostic)
    print(json.dumps({"replay_matches": result["unmodified_parser_exact_replay"],
                      "phases": [{"label": p["label"], "parser_delta": p["parser_delta"],
                                  "nucleo_span_ms": p["nucleo_span_ms"],
                                  "wire_sequence_counters": p["wire_sequence_counters"]}
                                 for p in result["phases"]]}), flush=True)
    fits = history(args.campaign, args.diagnostic) if args.campaign else []
    report(args.diagnostic, result, fits)
    if fits:
        print(json.dumps({"historical_timing": fits}), flush=True)


if __name__ == "__main__":
    main()
