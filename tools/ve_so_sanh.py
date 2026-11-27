#!/usr/bin/env python3
"""VẼ SO SÁNH 4 BỘ ĐIỀU KHIỂN (P01, P02, P06, P11) TRÊN TỪNG KỊCH BẢN — x650, ma trận thí nghiệm.

Đọc gazebo_logs/ma_tran_x650/x650_<bộ>_<kịch bản>_g<lần>.csv, ghi hình vào ma_tran_x650/do_thi/:
  1. so_sanh_<kịch bản>.png: ‖e_x‖(t) của 4 bộ (lần 1) trong đoạn bay quỹ đạo, đánh dấu lúc nhiễu bật;
     kịch bản tròn + yaw thêm hình Ψ(t) (sai số tư thế) để thấy yaw hội tụ.
  2. cot_rms.png: cột RMS (TB ± ĐLC của các lần) theo kịch bản × bộ điều khiển — bảng tổng kết một hình.
  3. uoc_luong.png: θ̄_x (P06, P11) và lực tích phân k_i·e_i (P02) theo thời gian ở kịch bản nhiễu — thấy bộ
     nào "nhận ra" nhiễu nhanh và đúng giá trị thật.
Cách dùng: python3 ~/ros2_px4_ws/src/p01/tools/ve_so_sanh.py
"""
import csv
import glob
import os
import re
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

THU_MUC = os.path.expanduser("~/báo/06_ket_qua (log, đồ thị Gazebo)/gazebo_logs/ma_tran_x650")
RA = os.path.join(THU_MUC, "do_thi")
BO = ["p01", "p02", "p06", "p11"]
MAU = {"p01": "#7f7f7f", "p02": "#1f77b4", "p06": "#2ca02c", "p11": "#d62728"}
TEN_BO = {"p01": "P01 (PD hình học)", "p02": "P02 (PID hình học)", "p06": "P06 (thích nghi, tư thế liên kết)",
          "p11": "P11 (thích nghi, tách yaw)"}
KICH_BAN = [  # (mã, tiêu đề, giá trị nhiễu thật để vẽ đường so sánh ở hình ước lượng)
    ("c1x05_dung", "Case I 0,5× — khai đúng khối lượng", None),
    ("c1x05_m26", "Case I 0,5× — khai thiếu 13% khối lượng (2,6/3,0 kg)", 0.4 * 9.81),
    ("c1x05_k11", "Case I 0,5× — motor yếu hơn mô hình 10%", 3.0 * 9.81 * (1 - 1 / 1.1)),
    ("c1x05_tai05", "Case I 0,5× — thêm tải 0,5 kg tại t = 15 s", 0.5 * 9.81),
    ("c1x05_gio", "Case I 0,5× — gió ngang 2 N tại t = 15 s", 2.0),
    ("tron_yaw", "Đường tròn + sai yaw ban đầu 179,4°", None),
]
TEP = re.compile(r"x650_(p01|p02|p06|p11)_(.+)_g(\d+)\.csv$")


def doc(p):
    with open(p, newline="") as f:
        r = csv.reader(f)
        cot = next(r)
        so = np.array([[float(v) for v in d] for d in r if len(d) == len(cot)])
    return {c: so[:, i] for i, c in enumerate(cot)}


def doan_bay(d, kb):
    """Mặt nạ đoạn bay quỹ đạo và thời điểm gốc (giống tong_hop_ma_tran.py)."""
    t, st = d["t"], d["stage"]
    bay = st == 3
    if kb == "tron_yaw":
        t0 = t[bay].min() + 3.0
        return (t >= t0) & (t <= t0 + 24.0), t0
    return bay, t[bay].min()


def main():
    os.makedirs(RA, exist_ok=True)
    tep = defaultdict(dict)
    for p in glob.glob(os.path.join(THU_MUC, "x650_*.csv")):
        k = TEP.search(os.path.basename(p))
        if k and "_LOI" not in p:
            tep[(k.group(2), k.group(1))][int(k.group(3))] = p

    rms = defaultdict(list)
    for (kb, bo), ds in tep.items():
        for p in ds.values():
            d = doc(p)
            m, _ = doan_bay(d, kb)
            rms[(kb, bo)].append(np.sqrt(np.mean((d["position_error"][m] * 100) ** 2)))

    # 1. ‖e_x‖(t) từng kịch bản
    for kb, tieu_de, _ in KICH_BAN:
        co = [b for b in BO if 1 in tep.get((kb, b), {})]
        if not co:
            continue
        hai = kb == "tron_yaw"
        fig, ax = plt.subplots(2 if hai else 1, 1, figsize=(10, 7 if hai else 4.5), sharex=True, squeeze=False)
        for b in co:
            d = doc(tep[(kb, b)][1])
            m, t0 = doan_bay(d, kb)
            tt = d["t"][m] - t0
            r = np.mean(rms[(kb, b)])
            ax[0, 0].plot(tt, d["position_error"][m] * 100, color=MAU[b], lw=1.4, label=f"{TEN_BO[b]} — RMS {r:.2f} cm")
            if hai:
                ax[1, 0].plot(tt, d["Psi"][m], color=MAU[b], lw=1.4, label=TEN_BO[b])
            if "dist_active" in d and (d["dist_active"] > 0.5).any() and b == co[0]:
                ax[0, 0].axvline(d["t"][d["dist_active"] > 0.5].min() - t0, color="k", ls="--", lw=1)
                ax[0, 0].text(d["t"][d["dist_active"] > 0.5].min() - t0, ax[0, 0].get_ylim()[1] * 0.9, " nhiễu bật", fontsize=9)
        ax[0, 0].set_ylabel("‖e_x‖ sai số vị trí (cm)")
        ax[0, 0].set_title(tieu_de + " (lần 1; RMS = TB các lần)")
        ax[0, 0].grid(alpha=0.3)
        ax[0, 0].legend(fontsize=8)
        if hai:
            ax[1, 0].set_ylabel("Ψ sai số tư thế")
            ax[1, 0].set_yscale("log")
            ax[1, 0].set_ylim(1e-5, 3)
            ax[1, 0].grid(alpha=0.3)
        ax[-1, 0].set_xlabel("thời gian kể từ đầu quỹ đạo (s)")
        fig.tight_layout()
        fig.savefig(os.path.join(RA, f"so_sanh_{kb}.png"), dpi=130)
        plt.close(fig)

    # 2. cột RMS
    kbs = [k for k, _, _ in KICH_BAN if any((k, b) in rms for b in BO)]
    fig, ax = plt.subplots(figsize=(11, 4.8))
    w = 0.2
    for i, b in enumerate(BO):
        y = [np.mean(rms[(k, b)]) if (k, b) in rms else np.nan for k in kbs]
        e = [np.std(rms[(k, b)], ddof=1) if len(rms.get((k, b), [])) > 1 else 0 for k in kbs]
        ax.bar(np.arange(len(kbs)) + (i - 1.5) * w, y, w, yerr=e, color=MAU[b], label=TEN_BO[b], capsize=2)
    ax.set_xticks(np.arange(len(kbs)))
    ax.set_xticklabels([k.replace("c1x05_", "Case I 0,5×\n").replace("tron_yaw", "tròn + yaw") for k in kbs], fontsize=9)
    ax.set_yscale("log")
    ax.set_ylabel("RMS ‖e_x‖ khi bay (cm, thang log)")
    ax.set_title("So sánh 4 bộ điều khiển trên X650 (trạng thái thật, TB ± ĐLC các lần bay)")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(RA, "cot_rms.png"), dpi=130)
    plt.close(fig)

    # 3. ước lượng nhiễu
    nhieu = [(k, td, v) for k, td, v in KICH_BAN if v is not None and any(1 in tep.get((k, b), {}) for b in BO)]
    if nhieu:
        fig, axs = plt.subplots(len(nhieu), 1, figsize=(10, 2.8 * len(nhieu)), squeeze=False)
        for ax, (kb, td, v) in zip(axs[:, 0], nhieu):
            truc = "x" if kb.endswith("gio") else "z"
            for b in ["p02", "p06", "p11"]:
                if 1 not in tep.get((kb, b), {}):
                    continue
                d = doc(tep[(kb, b)][1])
                m = (d["stage"] == 3) | (d["stage"] == 7)
                if b == "p02":
                    # lực tích phân P02 ≈ k_i·e_i; k_i mặc định = 1,0·gain_mass_kg = 3,0 (x650). Bỏ qua bão hòa σ (chưa chạm).
                    y = 3.0 * d[f"ei_{truc}"][m]   # cùng dấu với θ̄_x (cả hai vào A dưới dạng −k_i·e_i, −θ̄_x)
                else:
                    y = d[f"thx_{truc}"][m]
                ax.plot(d["t"][m], y, color=MAU[b], lw=1.4, label=TEN_BO[b])
            ax.axhline(v, color="k", ls="--", lw=1, label=f"nhiễu thật ≈ {v:.2f} N")
            ax.set_title(td, fontsize=10)
            ax.set_ylabel("ước lượng (N)")
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
        axs[-1, 0].set_xlabel("t (s) từ lúc cất cánh")
        fig.tight_layout()
        fig.savefig(os.path.join(RA, "uoc_luong.png"), dpi=130)
        plt.close(fig)
    print("Đã ghi hình vào", RA)


if __name__ == "__main__":
    main()
