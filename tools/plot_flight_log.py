#!/usr/bin/env python3
"""Vẽ log PX4/Gazebo của p01_offboard_hover; chỉ dùng các mẫu stage=3.

Đây là công cụ phân tích ngoại tuyến, không kết nối ROS/PX4 và không phát lệnh bay.
Mốc đầu/cuối di chuyển lấy từ cấu hình lần chạy qua đối số, không suy từ tên CSV.
"""

import argparse
import csv
import hashlib
import io
import json
import os
import platform
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # Xuất tệp, không cần màn hình hoặc mở cửa sổ GUI.
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D


COLUMNS = (
    "t,x,y,z,xd,yd,zd,position_error,Psi,f_requested,f_achieved,"
    "Mx,My,Mz,Mx_achieved,My_achieved,Mz_achieved,u0,u1,u2,u3,saturated,stage"
).split(",")
ACTUAL = "#0072B2"
REFERENCE = "#D55E00"
HOLD = "#00875A"


def load_flight(path):
    """Đọc đúng một bản dữ liệu trong bộ nhớ và kiểm tra trước khi vẽ."""
    raw = path.read_bytes()
    reader = csv.DictReader(io.StringIO(raw.decode("utf-8-sig")))
    missing = set(COLUMNS) - set(reader.fieldnames or [])
    if missing:
        raise ValueError(f"CSV thiếu cột: {sorted(missing)}")
    values = []
    for line, row in enumerate(reader, start=2):
        if None in row or any(row.get(key) in (None, "") for key in COLUMNS):
            raise ValueError(f"Hàng {line} thiếu/thừa giá trị; hãy chốt log trước khi vẽ.")
        values.append([float(row[key]) for key in COLUMNS])
    data = np.asarray(values, dtype=float)
    if data.size == 0 or not np.isfinite(data).all():
        raise ValueError("CSV rỗng hoặc chứa giá trị không hữu hạn.")
    all_cols = dict(zip(COLUMNS, data.T))
    stages = all_cols["stage"]
    if not np.isin(stages, np.arange(8)).all():
        raise ValueError("stage phải là số nguyên từ 0 tới 7 (0–4 bản cũ, 5–7 Arming/Spooling/Climbing).")
    active_idx = np.flatnonzero(stages == 3)
    if len(active_idx) < 2:
        raise ValueError("Cần ít nhất hai mẫu P01 đang có quyền điều khiển (stage=3).")
    if np.any(np.diff(active_idx) != 1):
        raise ValueError("Có nhiều đoạn Active rời nhau; cần chọn một phiên để phân tích.")
    cols = {key: val[active_idx] for key, val in all_cols.items()}
    if np.any(np.diff(cols["t"]) <= 0):
        raise ValueError("Thời gian trong đoạn Active không tăng nghiêm ngặt.")
    actual = np.column_stack([cols[key] for key in ("x", "y", "z")])
    desired = np.column_stack([cols[key] for key in ("xd", "yd", "zd")])
    error = actual - desired  # NED: thành phần thứ ba dương theo chiều xuống.
    norm = np.linalg.norm(error, axis=1)
    difference = float(np.max(np.abs(norm - cols["position_error"])))
    if difference > 1e-6:
        raise ValueError(f"Sai số tính lại không khớp cột position_error: {difference:.3g} m.")
    return cols, actual, desired, error, norm, {
        "source_sha256": hashlib.sha256(raw).hexdigest(),
        "source_bytes": len(raw),
        "all_rows": len(values),
        "stage_counts": {str(int(s)): int(np.sum(stages == s)) for s in np.unique(stages)},
        "last_recorded_stage": int(stages[-1]),
        "max_error_recomputation_difference_m": difference,
    }


def error_statistics(t, norm, mask):
    if not np.any(mask):
        return {"samples": 0}
    indices = np.flatnonzero(mask)
    peak = indices[int(np.argmax(norm[mask]))]
    return {
        "samples": int(np.sum(mask)),
        "time_first_s": float(t[indices[0]]),
        "time_last_s": float(t[indices[-1]]),
        "mean_m": float(np.mean(norm[mask])),
        "rms_m": float(np.sqrt(np.mean(norm[mask] ** 2))),
        "max_m": float(norm[peak]),
        "time_at_max_s": float(t[peak]),
    }


def phase_background(ax, start, end):
    """Dải nền là mốc cấu hình từ lệnh chạy, không phải phát hiện tự động."""
    ax.axvspan(start, end, color="#E8EDF2", zorder=0)
    for value in (start, end):
        ax.axvline(value, color="#697586", linewidth=0.8, linestyle=":", zorder=1)
    ax.grid(True, color="#D8DEE5", linewidth=0.6, alpha=0.8)
    ax.set_axisbelow(True)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.ticklabel_format(axis="y", style="plain", useOffset=False)


def heading(fig, title, subtitle):
    fig.suptitle(title, x=0.045, y=0.975, ha="left", fontsize=17, fontweight="bold")
    fig.text(0.045, 0.925, subtitle, fontsize=10, color="#445469")


def trajectory_figure(actual, desired, source_name):
    # Chỉ đổi dấu z khi HIỂN THỊ cao độ; không đổi dữ liệu gốc hoặc sai số NED.
    measured = actual * np.array([1.0, 1.0, -1.0])
    reference = desired * np.array([1.0, 1.0, -1.0])
    fig = plt.figure(figsize=(12, 8))
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    top = fig.add_subplot(1, 2, 2)
    heading(fig, "P01 / X500 — Quỹ đạo thực hiện và tham chiếu",
            "Case I đã điều chỉnh, tốc độ pha 0,25 · Chỉ mẫu stage=3 · Không làm mượt dữ liệu")
    ax3.plot(*measured.T, color=ACTUAL, linewidth=1.0, label="Phản hồi PX4")
    ax3.plot(*reference.T, color=REFERENCE, linewidth=1.5, linestyle="--", label="Tham chiếu")
    ax3.scatter(*reference[0], color=HOLD, s=45, depthshade=False)
    ax3.scatter(*reference[-1], color="#7B3294", marker="s", s=45, depthshade=False)
    ax3.set(xlabel="Bắc — x (m)", ylabel="Đông — y (m)", zlabel="Cao độ h = −z (m)")
    ax3.set_title("Không gian 3D", pad=18)
    ax3.view_init(elev=23, azim=-42)
    span = np.ptp(np.vstack([measured, reference]), axis=0)
    ax3.set_box_aspect(np.maximum(span, 0.1))
    top.plot(actual[:, 0], actual[:, 1], color=ACTUAL, linewidth=1.0)
    top.plot(desired[:, 0], desired[:, 1], color=REFERENCE, linewidth=1.5, linestyle="--")
    top.scatter(*desired[0, :2], color=HOLD, s=45, zorder=5)
    top.scatter(*desired[-1, :2], color="#7B3294", marker="s", s=45, zorder=5)
    top.set(xlabel="Bắc — x (m)", ylabel="Đông — y (m)", title="Nhìn từ trên: giữ đúng tỉ lệ hai trục")
    top.set_aspect("equal", adjustable="box")
    top.grid(True, color="#D8DEE5", linewidth=0.6)
    handles = [
        Line2D([], [], color=ACTUAL, label="Phản hồi PX4"),
        Line2D([], [], color=REFERENCE, linestyle="--", label="Tham chiếu"),
        Line2D([], [], color=HOLD, marker="o", linestyle="none", label="Điểm đầu tham chiếu"),
        Line2D([], [], color="#7B3294", marker="s", linestyle="none", label="Điểm cuối tham chiếu"),
    ]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.89), ncol=2, frameon=False)
    fig.text(0.045, 0.075, "Phản hồi là vị trí ước lượng của PX4, không phải trạng thái thật tuyệt đối của Gazebo.", fontsize=9)
    fig.text(0.045, 0.045, f"Hình 1 · Nguồn: {source_name} · h tính theo gốc cục bộ PX4, không phải khoảng cách tới sàn.", fontsize=8.5, color="#526171")
    fig.subplots_adjust(left=0.03, right=0.96, bottom=0.17, top=0.78, wspace=0.15)
    return fig


def position_figure(t, actual, desired, start, end, source_name):
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    heading(fig, "Vị trí phản hồi và tham chiếu theo thời gian",
            "Dải nền: đoạn di chuyển theo cấu hình · Trước đó giữ điểm đầu; sau đó giữ điểm cuối")
    for i, (ax, label, sign) in enumerate(zip(axes, ("Bắc x (m)", "Đông y (m)", "Cao độ h = −z (m)"), (1, 1, -1))):
        phase_background(ax, start, end)
        ax.plot(t, sign * actual[:, i], color=ACTUAL, linewidth=1.0, label="Phản hồi PX4")
        ax.plot(t, sign * desired[:, i], color=REFERENCE, linewidth=1.5, linestyle="--", label="Tham chiếu")
        ax.set_ylabel(label)
        ax.set_xlim(0, float(t[-1]))
    axes[0].legend(loc="upper right", ncol=2, frameon=False)
    axes[0].text(start + 0.5, 0.92, f"Di chuyển {start:g}–{end:g} s", transform=axes[0].get_xaxis_transform(), fontsize=9)
    axes[-1].set_xlabel("Thời gian kể từ nhận quyền Offboard (s)")
    fig.text(0.045, 0.035, f"Hình 2 · Nguồn: {source_name} · Chỉ stage=3; dữ liệu không lọc/làm mượt.", fontsize=8.5, color="#526171")
    fig.subplots_adjust(left=0.12, right=0.96, bottom=0.12, top=0.86, hspace=0.16)
    return fig


def error_figure(t, error, norm, summary, start, end, source_name):
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 9))
    heading(fig, "Sai số bám vị trí — ba thành phần và chuẩn vector",
            "Sai số = phản hồi − tham chiếu trong NED · Đơn vị cm · Thành phần z dương theo chiều xuống")
    for i, ax in enumerate(axes[:3]):
        phase_background(ax, start, end)
        ax.plot(t, 100 * error[:, i], color=ACTUAL, linewidth=0.9)
        limit = max(float(np.max(np.abs(error[:, i]))) * 115, 0.5)
        ax.set_ylim(-limit, limit)
        ax.axhline(0, color="#526171", linewidth=0.7)
        ax.set_ylabel(("e Bắc (cm)", "e Đông (cm)", "e Xuống (cm)")[i])
    ax = axes[-1]
    phase_background(ax, start, end)
    ax.plot(t, 100 * norm, color=ACTUAL, linewidth=0.9, label="Chuẩn sai số vị trí")
    for key, left, right, color, label in (
        ("movement", start, end, REFERENCE, "RMS đoạn di chuyển"),
        ("final_hold", end, float(t[-1]), HOLD, "RMS giữ điểm cuối"),
    ):
        stats = summary["segments"][key]
        if stats["samples"]:
            ax.hlines(100 * stats["rms_m"], left, right, color=color, linestyle="--", linewidth=1.8, label=f"{label}: {100 * stats['rms_m']:.2f} cm")
    peak = int(np.argmax(norm))
    ax.scatter(t[peak], 100 * norm[peak], color="#A6192E", s=25, zorder=5)
    ax.annotate(f"Cực đại {100 * norm[peak]:.2f} cm\nt = {t[peak]:.2f} s",
                xy=(t[peak], 100 * norm[peak]), xytext=(12, 8), textcoords="offset points", fontsize=8.5)
    ax.set_ylim(0, float(np.max(norm)) * 140)
    ax.set_ylabel("Chuẩn sai số (cm)")
    ax.set_xlabel("Thời gian kể từ nhận quyền Offboard (s)")
    ax.set_xlim(0, float(t[-1]))
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.37), ncol=3, frameon=False, fontsize=8)
    fig.text(0.045, 0.025, f"Hình 3 · Nguồn: {source_name} · RMS tính từ từng mẫu stage=3; dải nền {start:g}–{end:g} s.", fontsize=8.5, color="#526171")
    fig.subplots_adjust(left=0.12, right=0.96, bottom=0.15, top=0.86, hspace=0.16)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--motion-start", type=float, default=3.0)
    parser.add_argument("--motion-end", type=float, default=33.0)
    args = parser.parse_args()
    start, end = args.motion_start, args.motion_end
    if not (np.isfinite([start, end]).all() and 0 <= start < end):
        parser.error("Mốc di chuyển phải hữu hạn và 0 <= start < end.")
    source = args.csv.resolve()
    cols, actual, desired, error, norm, provenance = load_flight(source)
    t = cols["t"]
    masks = {"initial_hold": t < start, "movement": (t >= start) & (t < end), "final_hold": t >= end}
    if not all(np.any(mask) for mask in masks.values()):
        raise ValueError("Log chưa có đủ giữ đầu/di chuyển/giữ cuối theo các mốc đã chọn.")
    ref_range = np.ptp(desired[masks["final_hold"]], axis=0)
    if float(np.max(ref_range)) > 1e-6:
        raise ValueError("Tham chiếu còn thay đổi sau motion-end; hãy kiểm tra mốc cấu hình.")
    summary = {
        "source": str(source), **provenance,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "runtime": {"python": platform.python_version(), "numpy": np.__version__, "matplotlib": matplotlib.__version__},
        "selection": "stage == 3; one contiguous Active segment",
        "coordinates": {"data": "PX4 local NED, meters", "display_height": "h = -z_NED", "errors": "feedback - reference, NED"},
        "phase_boundaries_from_run_configuration_s": {"movement_start": start, "movement_end": end},
        "active": error_statistics(t, norm, np.ones(len(t), dtype=bool)),
        "segments": {key: error_statistics(t, norm, mask) for key, mask in masks.items()},
        "requested_force_range_N": [float(np.min(cols["f_requested"])), float(np.max(cols["f_requested"]))],
        "allocator_saturation_samples": int(np.count_nonzero(cols["saturated"])),
        "final_hold_reference_span_m": ref_range.tolist(),
    }
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "pdf.fonttype": 42, "axes.unicode_minus": True})
    figures = [
        ("01_quy_dao_3d", trajectory_figure(actual, desired, source.name)),
        ("02_vi_tri_theo_thoi_gian", position_figure(t, actual, desired, start, end, source.name)),
        ("03_sai_so_bam", error_figure(t, error, norm, summary, start, end, source.name)),
    ]
    with PdfPages(output / "P01_Gazebo_CaseI_01_Do_thi.pdf") as pdf:
        pdf.infodict()["Title"] = "P01 X500 — Case I điều chỉnh: quỹ đạo và sai số"
        for name, fig in figures:
            fig.savefig(output / f"{name}.png", dpi=240, facecolor="white")
            pdf.savefig(fig, facecolor="white")
            plt.close(fig)
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    lines = [
        "# Đồ thị P01 / Gazebo X500 — Case I điều chỉnh, lần 01", "",
        "Đọc các hình từ dữ liệu phản hồi PX4, không phải trạng thái thật tuyệt đối của Gazebo. Chỉ lấy stage=3, không lọc hoặc làm mượt dữ liệu.", "",
        "- [PDF ba trang](./P01_Gazebo_CaseI_01_Do_thi.pdf)",
        "- [Hình 1: quỹ đạo 3D và hình chiếu ngang](./01_quy_dao_3d.png)",
        "- [Hình 2: vị trí phản hồi/tham chiếu theo thời gian](./02_vi_tri_theo_thoi_gian.png)",
        "- [Hình 3: sai số theo từng trục NED và chuẩn sai số](./03_sai_so_bam.png)",
        "- [Số liệu và thông tin nguồn](./summary.json)", "",
        f"Nguồn: [{source.name}]({os.path.relpath(source, output)}). SHA-256: `{provenance['source_sha256']}`.", "",
        f"Dải nền là đoạn di chuyển {start:g}–{end:g} s theo cấu hình lần chạy. Sau {end:g} s, ba tọa độ tham chiếu cố định trong dữ liệu. Cao độ hiển thị h=-z_NED; sai số trục thứ ba vẫn giữ dấu NED (dương xuống).", "",
        "| Đoạn | Mẫu | Trung bình (cm) | RMS (cm) | Lớn nhất (cm) |", "|---|---:|---:|---:|---:|",
    ]
    for key, label in (("initial_hold", "Giữ điểm đầu"), ("movement", "Di chuyển"), ("final_hold", "Giữ điểm cuối")):
        stat = summary["segments"][key]
        lines.append(f"| {label} | {stat['samples']} | {100 * stat['mean_m']:.3f} | {100 * stat['rms_m']:.3f} | {100 * stat['max_m']:.3f} |")
    lines += ["", f"RMS là căn trung bình bình phương chuẩn sai số theo các mẫu ghi. Số mẫu bão hòa theo bộ phân bổ: {summary['allocator_saturation_samples']}. Các cột lực/mômen achieved là giá trị tính lại từ phân bổ, không phải đo lực rotor.", "",
              f"Bản dữ liệu được vẽ có stage cuối là {summary['last_recorded_stage']}; hình không xác nhận nguyên nhân/kết quả thoát Offboard. Đây là Case I điều chỉnh cho X500, chưa phải tái hiện nguyên bản CDC.", "",
              "## Tạo lại trên Ubuntu", "", "Chạy từ thư mục gốc báo, với NumPy và Matplotlib đã có trong môi trường Python:", "", "```bash",
              f"/usr/bin/python3 p01/tools/plot_flight_log.py --csv {os.path.relpath(source, Path.cwd())} --output {os.path.relpath(output, Path.cwd())} --motion-start {start:g} --motion-end {end:g}",
              "```", "", f"Môi trường đã chạy: Python {platform.python_version()}, NumPy {np.__version__}, Matplotlib {matplotlib.__version__}. Bộ điều khiển ROS 2/PX4 không bị sửa hoặc khởi động bởi công cụ vẽ.", ""]
    (output / "README.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"output": str(output), "active_samples": len(t), "movement": summary["segments"]["movement"], "final_hold": summary["segments"]["final_hold"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
