#!/usr/bin/env python3
"""TỔNG HỢP MA TRẬN THÍ NGHIỆM x650 (kết quả của chay_ma_tran.sh).

Đọc gazebo_logs/ma_tran_x650/x650_<bdk>_<kịch bản>_g<lần>.csv, tính chỉ số trên ĐOẠN BAY QUỸ ĐẠO:
  - case1 (c1_* = 1×, c1x05_* = 0,5×): stage 3 (sau cất cánh tới trước bay về), giống các bảng X500 trước đây.
  - tron_yaw: 24 s kể từ lúc bắt đầu quỹ đạo (stage 3 + 3 s chờ).
Chỉ số: RMS và max ‖e_x‖ (cm), số mẫu bão hòa, thời điểm cuối cùng Ψ ≥ 0,01 (yaw/tư thế hội tụ hẳn), ‖θ̄_R‖ max.
  Dao động tư thế cuối chuyến (bổ sung 25/11): góc sai tư thế lớn nhất θ = arccos(1 − Ψ) (độ) và độ lệch chuẩn Mz (N.m)
  trên 14 s cuối của đoạn đo. Lý do: Ψ < 0,01 (θ < 8,1°) vẫn để lọt dao động yaw của P06 (θ tới khoảng 6,5°).
Ghi: tong_hop_tung_lan.csv, tong_hop_trung_binh.csv và in bảng TB ± ĐLC ra màn hình.
Chỉ dùng thư viện chuẩn + numpy.
"""
import csv
import glob
import os
import re
from collections import defaultdict

import numpy as np

THU_MUC = os.environ.get("MA_TRAN_DIR", os.path.expanduser("~/báo/06_ket_qua (log, đồ thị Gazebo)/gazebo_logs/ma_tran_x650"))
MAU = re.compile(r"x650_(p01|p02|p06c|p06|p11)_(c1(?:x\d+)?_(?:dung|m26|k11|tai05|gio|mom)(?:_ekf)?|tron_yaw(?:_ekf)?)_g(\d+)\.csv$")


def doc(duong_dan):
    with open(duong_dan, newline="") as f:
        r = csv.reader(f)
        cot = next(r)
        so = np.array([[float(v) for v in dong] for dong in r if len(dong) == len(cot)])
    d = {c: so[:, i] for i, c in enumerate(cot)}
    # Chuyến chạy bằng EKF2 (state_source_gazebo = 0): đo bằng sai số THẬT (cột gt_pos_err), giữ sai số theo EKF2
    # ở cột position_error_ekf. Chuyến cũ không có cột gt_* thì giữ nguyên.
    if "gt_pos_err" in d and "state_source_gazebo" in d and (d["state_source_gazebo"] < 0.5).all():
        d["position_error_ekf"] = d["position_error"]
        d["position_error"] = d["gt_pos_err"]
    return d


def chi_so(d, kb):
    t, st = d["t"], d["stage"]
    bay = st == 3
    if kb == "tron_yaw":
        t0 = t[bay].min() + 3.0
        m = (t >= t0) & (t <= t0 + 24.0)
    else:
        t0 = t[bay].min()
        m = bay
    e = d["position_error"][m] * 100.0
    psi = d["Psi"][m]
    tt = t[m] - t0
    xau = np.where(psi >= 0.01)[0]
    cuoi = tt >= tt.max() - 14.0
    out = {
        "rms_cm": float(np.sqrt(np.mean(e ** 2))),
        "max_cm": float(e.max()),
        "bao_hoa": int(d["saturated"][m].sum()),
        "goc_cuoi_max_deg": float(np.degrees(np.arccos(np.clip(1.0 - psi[cuoi], -1.0, 1.0))).max()),
        "Mz_cuoi_dlc": float(d["Mz"][m][cuoi].std()),
        "psi_hoi_tu_s": float(tt[xau[-1]]) if len(xau) else 0.0,
        "thR_max": float(np.linalg.norm(np.column_stack([d["thR_x"][m], d["thR_y"][m], d["thR_z"][m]]), axis=1).max())
        if "thR_x" in d else float("nan"),
        "rms_5s_sau_nhieu_cm": float("nan"), "max_5s_sau_nhieu_cm": float("nan"), "rms_sau_5s_cm": float("nan"),
        "thx_cuoi": float("nan"), "thR_cuoi": float("nan"),
    }
    # Kịch bản có nhiễu ngoài đổi giữa chuyến: tách 5 s ngay sau khi nhiễu bật (hồi phục) và phần còn lại của quỹ đạo.
    if "dist_active" in d and (d["dist_active"] > 0.5).any():
        t_on = t[d["dist_active"] > 0.5].min()
        t_het = t[m].max()
        e_all = d["position_error"] * 100.0
        w1 = m & (t >= t_on) & (t < t_on + 5.0)
        w2 = m & (t >= t_on + 5.0) & (t <= t_het)
        if w1.any():
            out["rms_5s_sau_nhieu_cm"] = float(np.sqrt(np.mean(e_all[w1] ** 2)))
            out["max_5s_sau_nhieu_cm"] = float(e_all[w1].max())
        if w2.any():
            out["rms_sau_5s_cm"] = float(np.sqrt(np.mean(e_all[w2] ** 2)))
        # Ước lượng nhiễu cuối đoạn đo (5 s cuối): ‖θ̄_x‖ (N), ‖θ̄_R‖ (N.m) — so với nhiễu thật đã đặt.
        w3 = m & (t >= t_het - 5.0)
        if "thR_x" in d and w3.any():
            out["thx_cuoi"] = float(np.linalg.norm(np.column_stack([d["thx_x"][w3], d["thx_y"][w3], d["thx_z"][w3]]), axis=1).mean())
            out["thR_cuoi"] = float(np.linalg.norm(np.column_stack([d["thR_x"][w3], d["thR_y"][w3], d["thR_z"][w3]]), axis=1).mean())
    return out


def main():
    hang = []
    for p in sorted(glob.glob(os.path.join(THU_MUC, "x650_*.csv"))):
        k = MAU.search(os.path.basename(p))
        if not k:
            continue
        bdk, kb, lan = k.group(1), k.group(2), int(k.group(3))
        try:
            c = chi_so(doc(p), kb)
        except Exception as loi:  # tệp hỏng/chưa đủ: báo và bỏ qua
            print("Bỏ qua", os.path.basename(p), loi)
            continue
        hang.append({"bdk": bdk, "kich_ban": kb, "lan": lan, **c})
    if not hang:
        print("Chưa có CSV nào trong", THU_MUC)
        return
    ten = ["bdk", "kich_ban", "lan", "rms_cm", "max_cm", "bao_hoa", "goc_cuoi_max_deg", "Mz_cuoi_dlc", "psi_hoi_tu_s", "thR_max",
           "rms_5s_sau_nhieu_cm", "max_5s_sau_nhieu_cm", "rms_sau_5s_cm",
           "thx_cuoi", "thR_cuoi"]
    with open(os.path.join(THU_MUC, "tong_hop_tung_lan.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=ten)
        w.writeheader()
        w.writerows(hang)

    nhom = defaultdict(list)
    for h in hang:
        nhom[(h["kich_ban"], h["bdk"])].append(h)
    tb = []
    for (kb, bdk), ds in sorted(nhom.items()):
        r = np.array([h["rms_cm"] for h in ds])
        tb.append({
            "kich_ban": kb, "bdk": bdk, "so_lan": len(ds),
            "rms_tb_cm": r.mean(), "rms_dlc_cm": r.std(ddof=1) if len(r) > 1 else 0.0,
            "max_tb_cm": np.mean([h["max_cm"] for h in ds]),
            "bao_hoa_tb": np.mean([h["bao_hoa"] for h in ds]),
            "psi_hoi_tu_tb_s": np.mean([h["psi_hoi_tu_s"] for h in ds]),
            "goc_cuoi_max_tb_deg": np.mean([h["goc_cuoi_max_deg"] for h in ds]),
            "Mz_cuoi_dlc_tb": np.mean([h["Mz_cuoi_dlc"] for h in ds]),
            "rms_5s_sau_nhieu_tb_cm": np.mean([h["rms_5s_sau_nhieu_cm"] for h in ds]),
            "max_5s_sau_nhieu_tb_cm": np.mean([h["max_5s_sau_nhieu_cm"] for h in ds]),
            "rms_sau_5s_tb_cm": np.mean([h["rms_sau_5s_cm"] for h in ds]),
            "thx_cuoi_tb": np.mean([h["thx_cuoi"] for h in ds]),
            "thR_cuoi_tb": np.mean([h["thR_cuoi"] for h in ds]),
        })
    with open(os.path.join(THU_MUC, "tong_hop_trung_binh.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(tb[0].keys()))
        w.writeheader()
        for h in tb:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in h.items()})

    print(f"{'kịch bản':12s} {'bộ':5s} {'n':>2s} {'RMS TB ± ĐLC (cm)':>18s} {'max TB':>7s} {'bão hòa':>8s} {'Ψ hội tụ (s)':>12s} {'θ cuối (°)':>10s} {'σMz cuối':>9s}"
          f" {'5s sau nhiễu RMS/max':>21s} {'sau đó RMS':>11s} {'‖θ̄x‖/‖θ̄R‖ cuối':>17s}")
    for h in tb:
        nh = "" if np.isnan(h["rms_5s_sau_nhieu_tb_cm"]) else \
            f" {h['rms_5s_sau_nhieu_tb_cm']:10.2f}/{h['max_5s_sau_nhieu_tb_cm']:<10.2f} {h['rms_sau_5s_tb_cm']:11.2f}" + \
            ("" if np.isnan(h["thR_cuoi_tb"]) else f" {h['thx_cuoi_tb']:8.2f}/{h['thR_cuoi_tb']:<8.3f}")
        print(f"{h['kich_ban']:12s} {h['bdk']:5s} {h['so_lan']:2d} {h['rms_tb_cm']:9.2f} ± {h['rms_dlc_cm']:5.2f}"
              f" {h['max_tb_cm']:7.2f} {h['bao_hoa_tb']:8.0f} {h['psi_hoi_tu_tb_s']:12.1f} {h['goc_cuoi_max_tb_deg']:10.2f} {h['Mz_cuoi_dlc_tb']:9.3f}{nh}")
    print("Đã ghi tong_hop_tung_lan.csv, tong_hop_trung_binh.csv trong", THU_MUC)


if __name__ == "__main__":
    main()
