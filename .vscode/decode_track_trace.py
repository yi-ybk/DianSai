import argparse
import csv
import json
import struct
from collections import Counter
from pathlib import Path


TRACE_MAGIC = 0x314B5254
TRACE_VERSION = 3
TRACE_CAPACITY = 480
HEADER_STRUCT = struct.Struct("<15I")
SAMPLE_STRUCT = struct.Struct("<HHBB5h4B")
SUMMARY_SIZE = 268

STATUS_LINE_LOST = 0x10
STATUS_OUTER_SENSOR = 0x20
STATUS_TURN_SATURATED = 0x40
STATUS_WHEEL_SCALED = 0x80


def decode_trace(path: Path):
    raw = path.read_bytes()
    if len(raw) < HEADER_STRUCT.size:
        raise ValueError(f"trace dump is too short: {len(raw)} bytes")

    values = HEADER_STRUCT.unpack_from(raw)
    names = (
        "magic",
        "version",
        "sample_size",
        "capacity",
        "count",
        "write_index",
        "sample_period_ms",
        "mode",
        "active",
        "complete",
        "overflow",
        "start_tick_ms",
        "last_sample_tick_ms",
        "stop_tick_ms",
        "capture_call_count",
    )
    header = dict(zip(names, values))
    if header["magic"] != TRACE_MAGIC:
        raise ValueError(
            f"trace magic mismatch: 0x{header['magic']:08X}; "
            "the board may have reset or the wrong firmware may be running"
        )
    if header["version"] != TRACE_VERSION:
        raise ValueError(f"unsupported trace version: {header['version']}")
    if header["sample_size"] != SAMPLE_STRUCT.size:
        raise ValueError(
            f"sample size mismatch: firmware={header['sample_size']}, "
            f"decoder={SAMPLE_STRUCT.size}"
        )
    if header["capacity"] != TRACE_CAPACITY:
        raise ValueError(f"unexpected trace capacity: {header['capacity']}")
    if header["count"] > header["capacity"]:
        raise ValueError(
            f"invalid sample count: {header['count']} > {header['capacity']}"
        )
    required_size = HEADER_STRUCT.size + header["capacity"] * SAMPLE_STRUCT.size
    if len(raw) != required_size:
        raise ValueError(
            f"trace dump size mismatch: got {len(raw)}, expected {required_size}"
        )

    samples = []
    offset = HEADER_STRUCT.size
    if header["overflow"]:
        physical_indices = list(range(header["write_index"], header["capacity"]))
        physical_indices += list(range(0, header["write_index"]))
    else:
        physical_indices = list(range(header["count"]))
    for index, physical_index in enumerate(physical_indices):
        (
            elapsed_ms,
            distance_mm,
            black_mask,
            status,
            normalized_error_x1000,
            control_error_x1000,
            pid_output_mradps,
            command_forward_mmps,
            command_turn_mradps,
            raw_black_mask,
            selected_black_mask,
            black_run_count,
            selection_state,
        ) = SAMPLE_STRUCT.unpack_from(
            raw, offset + physical_index * SAMPLE_STRUCT.size
        )
        samples.append(
            {
                "index": index,
                "elapsed_ms": elapsed_ms,
                "distance_m": distance_mm / 1000.0,
                "black_mask": black_mask,
                "black_mask_hex": f"0x{black_mask:02X}",
                "black_count": status & 0x0F,
                "line_lost": bool(status & STATUS_LINE_LOST),
                "outer_sensor_only": bool(status & STATUS_OUTER_SENSOR),
                "turn_saturated": bool(status & STATUS_TURN_SATURATED),
                "wheel_speed_scaled": bool(status & STATUS_WHEEL_SCALED),
                "normalized_error": normalized_error_x1000 / 1000.0,
                "control_error": control_error_x1000 / 1000.0,
                "pid_output_radps": pid_output_mradps / 1000.0,
                "command_forward_mps": command_forward_mmps / 1000.0,
                "command_turn_radps": command_turn_mradps / 1000.0,
                "raw_black_mask": raw_black_mask,
                "raw_black_mask_hex": f"0x{raw_black_mask:02X}",
                "selected_black_mask": selected_black_mask,
                "selected_black_mask_hex": f"0x{selected_black_mask:02X}",
                "black_run_count": black_run_count,
                "direction_change_count": selection_state & 0x0F,
                "raw_was_filtered": bool(selection_state & 0x10),
                "direction_was_held": bool(selection_state & 0x20),
                "direction_was_confirmed": bool(selection_state & 0x40),
                "jump_was_rejected": bool(selection_state & 0x80),
            }
        )
    return header, samples


def decode_summary(path: Path):
    raw = path.read_bytes()
    if len(raw) != SUMMARY_SIZE:
        raise ValueError(
            f"track summary size mismatch: got {len(raw)}, expected {SUMMARY_SIZE}"
        )

    summary = {}
    offset = 0

    def take(fmt, names):
        nonlocal offset
        values = struct.unpack_from("<" + fmt, raw, offset)
        offset += struct.calcsize("<" + fmt)
        summary.update(zip(names, values))

    take("5I", ("active", "mode", "update_count", "elapsed_ms", "stop_reason"))
    take("2I", ("black_mask", "black_count"))
    take("3f", ("normalized_error", "last_nonzero_error", "line_lost_time_s"))
    take("6f", (
        "pid_p_out", "pid_i_out", "pid_d_out", "pid_output",
        "base_turn_speed_radps", "active_turn_limit_radps",
    ))
    take("7f", (
        "target_forward_speed_mps", "target_turn_speed_radps",
        "command_forward_speed_mps", "command_turn_speed_radps",
        "measured_forward_speed_mps", "measured_turn_speed_radps",
        "wheel_speed_scale",
    ))
    take("4f", (
        "left_target_speed_mps", "left_measured_speed_mps",
        "right_target_speed_mps", "right_measured_speed_mps",
    ))
    take("10f", (
        "max_abs_error", "max_abs_pid_d_out",
        "max_abs_base_turn_speed_radps", "max_abs_command_turn_speed_radps",
        "min_command_forward_speed_mps", "max_command_forward_speed_mps",
        "min_wheel_speed_scale", "max_left_speed_error_mps",
        "max_right_speed_error_mps", "max_line_lost_time_s",
    ))
    take("7I", (
        "line_lost_active", "line_lost_event_count", "line_lost_sample_count",
        "outer_sensor_sample_count", "large_error_sample_count",
        "turn_saturation_sample_count", "max_direction_change_count",
    ))
    sensor_hit_count = list(struct.unpack_from("<8I", raw, offset))
    offset += struct.calcsize("<8I")
    black_count_histogram = list(struct.unpack_from("<9I", raw, offset))
    offset += struct.calcsize("<9I")
    take("2I", ("max_error_elapsed_ms", "max_error_black_mask"))
    take("4f", (
        "max_error_signed", "max_error_forward_speed_mps",
        "max_error_turn_speed_radps", "final_distance_m",
    ))
    summary["sensor_hit_count"] = sensor_hit_count
    summary["black_count_histogram"] = black_count_histogram
    summary["black_mask_hex"] = f"0x{summary['black_mask'] & 0xFF:02X}"
    summary["max_error_black_mask_hex"] = (
        f"0x{summary['max_error_black_mask'] & 0xFF:02X}"
    )
    if offset != SUMMARY_SIZE:
        raise ValueError(f"internal summary decoder offset mismatch: {offset}")
    return summary


def analyze_samples(samples):
    if not samples:
        return {"sample_count": 0}

    mask_counts = Counter(sample["black_mask_hex"] for sample in samples)
    worst_errors = sorted(
        samples, key=lambda sample: abs(sample["normalized_error"]), reverse=True
    )[:10]
    return {
        "sample_count": len(samples),
        "duration_ms": samples[-1]["elapsed_ms"],
        "distance_m": samples[-1]["distance_m"],
        "line_lost_samples": sum(sample["line_lost"] for sample in samples),
        "outer_sensor_samples": sum(
            sample["outer_sensor_only"] for sample in samples
        ),
        "turn_saturated_samples": sum(
            sample["turn_saturated"] for sample in samples
        ),
        "wheel_speed_scaled_samples": sum(
            sample["wheel_speed_scaled"] for sample in samples
        ),
        "max_abs_error": max(abs(sample["normalized_error"]) for sample in samples),
        "filtered_raw_samples": sum(sample["raw_was_filtered"] for sample in samples),
        "direction_held_samples": sum(sample["direction_was_held"] for sample in samples),
        "direction_confirmed_samples": sum(sample["direction_was_confirmed"] for sample in samples),
        "jump_rejected_samples": sum(sample["jump_was_rejected"] for sample in samples),
        "black_mask_counts": dict(mask_counts.most_common()),
        "worst_error_samples": worst_errors,
    }


def main():
    parser = argparse.ArgumentParser(description="Decode MSPM0 track RAM trace")
    parser.add_argument("trace_bin", type=Path)
    parser.add_argument("summary_bin", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()

    header, samples = decode_trace(args.trace_bin)
    summary = decode_summary(args.summary_bin)
    analysis = analyze_samples(samples)

    csv_path = args.output_prefix.with_suffix(".csv")
    json_path = args.output_prefix.with_suffix(".json")
    if samples:
        with csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=samples[0].keys())
            writer.writeheader()
            writer.writerows(samples)
    else:
        csv_path.write_text("", encoding="utf-8-sig")

    report = {
        "trace_header": header,
        "track_summary": summary,
        "trace_analysis": analysis,
    }
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print(
        f"Decoded {len(samples)} samples, mode={header['mode']}, "
        f"active={header['active']}, complete={header['complete']}, "
        f"overflow={header['overflow']}"
    )
    print(f"CSV: {csv_path}")
    print(f"JSON: {json_path}")


if __name__ == "__main__":
    main()
