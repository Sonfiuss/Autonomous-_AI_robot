---
id: 2026-07-11_stereo-accuracy
status: testing
module: stereo-camera
started: 2026-07-11
---

## Task
Tăng độ chính xác stereo camera (rig cố định): recalibrate + đo kiểm bằng thước dây + thử nghiệm matching, dải mục tiêu 0.3–2m.

## Input
- User: stereo nhiễu trên hàng ngàn điểm, đo khoảng cách chưa đạt kỳ vọng → golden points ra đời như giải pháp đối phó. Gác plan free-move-merge (design lưu tại agent/description/deepmap_free_move_merge.md), focus solution 2: tăng tin cậy stereo.
- Root cause: calib từ 2 cặp chessboard cùng ~0.9m trên màn laptop, RMS 1.897px, distortion=0, fx ±15%, chưa từng kiểm bằng thước.
- User có máy in (checkerboard); phần cứng giữ nguyên (baseline 5.4cm nghiêng 27°, toe-in 2.14°).
- Approved plan: C:\Users\admin\.claude\plans\now-i-had-deepmap-abundant-octopus.md

## Expected output
- Bộ tool đo: epipolar_check.py (dy p50/p90), depth_eval.py (Z vs thước dây, --selftest ±0.1px).
- Calib mới versioned (không ghi đè), pass: stereo RMS <0.5px, dy p90 <0.5px, |Z err| ≤3% @0.4–1.6m.
- fb_measured thực nghiệm ghi vào yml, stereo_ruler ưu tiên đọc.
- Bảng so sánh golden-tuned vs SGBM-calibrated (quyết định bằng số liệu).
- Replay walk shots: n_inliers từ 3 → ≥50% n_golden, tường 2m đọc 1.9–2.1m.

## Plan
- [x] A. Eval harness: epipolar_check.py + depth_eval.py + capture_eval.sh; selftest pass; baseline số liệu với calib hiện tại
- [x] B. make_checkerboard.py (9x6, 25mm, A4) + nâng cấp stereo_calibrate_2view.py (per-cam K, k1/k2, pp free, per-view rejection, versioned yml + coverage heatmap)
- [x] C. depth_eval.py --fit-fb/--write-fb → fb_measured vào yml; stereo_ruler.load_rectify ưu tiên fb_measured + đọc schema mới
- [x] D. --sweep-golden + sgbm_probe.py (SGBM trên calib thật, LR-check thủ công, WLS nếu có contrib) + lock_exposure.sh
- [ ] E. Replay A/B walk shots trên Jetson + ACCURACY doc (HOÃN theo user 13/07 — chờ chuẩn hóa servo pitch xong)
- [x] F. pitch_from_board.py: đo pitch/yaw/roll camera từ board tường (solvePnP, +pitch=cúi) + dy per góc; selftest PASS 0.0000°; smoke test 6 cặp OK (pitchL≈0-2° khớp "nhìn thẳng", L−R≈−4..−5°)
- [ ] G. Servo pitch sweep (user chụp, robot ĐỨNG YÊN chỉ servo quay): p00/p10/p20/p30 → pitch_from_board → quyết định 1 calib hay per-angle + bảng servo cmd→deg
- [ ] Capture sessions (user, Jetson): session calib 20–25 cặp nghiêng ±30° phủ 4 góc (servo 0°); sweep session bước G; (đã có: eval 6 khoảng cách, ô=25mm)

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
- [--:--] step A: calib_io.py + epipolar_check.py + depth_eval.py + capture_eval.sh created | info: cornerSubPix refine needed on top of SB detector (SB pixel-locks +-0.23px at quarter-px phase); selftest PASS board 0.027px, roi 0.018px
- [--:--] step A: baseline recorded (baseline_epipolar_20260711.csv) | info: dy p50=1.55px p90=3.28px on the calib's OWN chessboard pairs (in-sample) -> golden_points +-1px row search misses most true matches
- [--:--] step A: PHAT HIEN left.jpg (27/06) + righ.jpg (05/07) chup cach nhau 8 NGAY - khong phai cap stereo that | risk: moi test offline dung cap mac dinh nay (ket qua 39golden/3inl ngay 09-10/07) deu tinh stereo tren 2 thoi diem khac nhau -> can chup lai cap chuan | info: capture_pair.sh da va: ghi .tmp, chi promote khi CA HAI camera OK
- [--:--] step B: make_checkerboard (PDF A4 9x6 25mm + ruler bar) + calibrate rewrite; synth test PASS: fx sai 0.03%, baseline 0.05%, goc +-0.02deg, rms 0.074px | info: find_board = SB + fallback classic + cornerSubPix (SB pixel-lock +-0.23px) + canonical order
- [--:--] step C: fb_measured round-trip verified (write_fb -> ca stereo_ruler lan calib_io uu tien fb_measured; backup yml vao calib/archive/ truoc khi ghi)
- [--:--] step D: sweep-golden (32 config) + sgbm_probe chay dung tren synth session (Zerr <=0.1% golden, <=0.73% sgbm, ground truth fb=60.69 khop) | info: Windows py khong co ximgproc -> sgbm_probe dung LR-check thu cong (flip 2 chieu), tu dong bat WLS neu Jetson co contrib
- [--:--] status -> testing: cho user in board + 2 capture session tren Jetson
- [22:30] eval session NHAN: 6 cap left/right_{81,82,97,112,120,180} (12/07 ~22:00, z=cm trong ten file) | info: board THAT la 19x10 inner (20x11 o) tren cua — KHONG phai PDF 9x6 25mm; detect 190/190 ca 12 anh; thu tu corner L/R NHAT QUAN (khong flip, cos=+0.999) -> calib an toan
- [22:40] baseline calib cu tren cap THAT: epipolar dy board p50 22-123px (dy tho ~-85px = baseline nghieng); depth_eval Z err -29%..-51%, fb_i troi 85->124 | info: fit-fb can d0=+41.7px, spread(d*z)=38.8 -> calib cu KHONG cuu duoc bang scale, bat buoc recalib
- [22:45] risk: cap _120 kha nghi (disparity 92.5 > _112 82.8 du xa hon; L/R ghi cach 35s — co the robot/board dich chuyen giua 2 lan grab) -> nen chup lai hoac loai
- [23:05] xac minh _120: kich thuoc board trong CA 2 anh khop 1.22m (khong phai nham nhan, board khong doi cho) NHUNG disparity +8px va dy tho -94.6px lech trend -> RIG xoay/xe dich giua 2 lan grab (L/R chup roi nhau 35s; ~0.4 deg yaw du tao +8px) | risk: TAT CA 6 cap deu chup L/R roi nhau 9-48s, chi _120 lo ro — session calib phai grab dong thoi 1 lenh capture_pair.sh
- [22:50] calibrate BLOCKED: script doi >=12 cap; 6 cap eval frontal chua du | info: can session calib 20-25 cap nghieng +-30deg phu 4 goc, pattern 19x10, + DO KICH THUOC O (mm) bang thuoc vi board nay khong phai board in
- [23:15] user xac nhan chup lan luot la chu dich, thong so dung -> tiep tuc voi 6 cap. Thu calibrate day du (driver bypass gate 8 view): SUY BIEN frontal (fx no 2560, k2=+53, f*B=0) — dung ly thuyet Zhang, board frontal khong rang buoc focal
- [23:25] plan-B calib: fx neo bang thuoc day (pitch*z/sq: fx_l=1023 fx_r=1032, spread +-3% << +-15% cu), pp=center dist=0, stereoCalibrate FIX_INTRINSIC -> R(tilt+5.7 yaw+0.6 roll-2.7), |T| pin 5.4cm | info: _112 loai khoi R,T (dy -13.7px khi held-out -> rig dich chuyen giua 2 lan grab cua no); yml -> calib/stereo_rectify_20260712.yml (versioned, CHUA promote)
- [23:30] epipolar voi yml moi: 5 cap sach dy p50 0.5-2.2px p90 1.3-3.4px (cu: 22-123px) — chua dat gate 0.5px nhung tot len 10-20x; residual +-2px = noise chup lan luot
- [23:32] FIX BUG depth_eval.py:371: write_fb ghi fb_median (model offset=0) kem d0 cua lstsq -> Z bias +d0/d; sua thanh ghi fb_lstsq khi |d0|>0.3. Khoi phuc yml tu backup, ghi lai fb_measured=62.766 + fb_offset_px=+2.04 (fit 5 cap sach)
- [23:35] KET QUA Z vs thuoc: 5 cap sach +2.2/-2.4/+0.9/-0.8/+0.3% -> DAT gate <=3% (0.8-1.8m); _112 held-out +4.0% | info: fb 62.77 vs pin-baseline nominal 55.48 -> goi y sq that ~23mm hoac baseline ~6.1cm — do o that de chot
- [23:45] A/B sgbm_probe 3 shot 15:06: yml moi cov 6-8% golden 30-40 vs cu 12-14%/50 — KHONG ket luan duoc (khong ground truth, vien anh warp khac, dist=0 sai o ria); can calib session 20-25 cap nghieng de uoc luong distortion + danh gia lai
- [.13] user duyet PROMOTE: archive stereo_rectify.yml cu (calib/archive/stereo_rectify_ACTIVE_pre20260713_*.yml) -> copy v2 (20260713) thanh ACTIVE; verify load OK (per-camera, fb_used=61.289 fb_measured, fx_rect=1045.4). Bao cao artifact: https://claude.ai/code/artifact/1310644e-96cb-44c2-b6e3-b1d0581539f1
- [.13] user do lai: 4 o NGANG = 9.0cm, 4 o DOC = 10.4cm -> o = 22.5x26.0mm (ty le 1.156); ket hop ty le pixel 1.101 -> PIXEL KHONG VUONG fy/fx~0.95 (tape 0.955/0.957, Zhang 0.944/0.940 — 2 phuong phap doc lap trung nhau tren ca 2 cam). Sweep servo user cho tam hoan.
- [.13] plan-B v2 -> calib/stereo_rectify_20260713.yml: K fx!=fy neo thuoc day (L 1089/1040, R 1098/1051), o that 22.5x26, |T| square-trusted 5.54cm (~do tay 5.4, khong pin), R tilt+5.54 roll-2.76; fb_measured=61.29 d0 chi +0.27px (v1 can +2.04 -> model v2 vat ly hon, scale correction chi x1.058) | KET QUA: Z err 5 cap sach +1.8/-3.2/+0.7/-0.7/-1.2% (~gate 3%, _82 lech nhe — cap co gap L/R 48s), epipolar nhu v1 (p50 0.57-2.0px). KHUYEN DUNG v2 thay v1 cho anh tu 12/07 toi tro di.
- [.13] user dinh chinh: _82/_120/_180 chup o goc lech board-camera (yaw -43..+26 do PnP xac nhan) -> data CO da dang yaw, chan doan "toan frontal" truoc do SAI; ly do Zhang no la khac
- [.13] PHAT HIEN BOARD: (1) o KHONG vuong — pitch doc/ngang = 1.101 (ca 2 cam trung nhau -> loi board khong phai cam); (2) board hoi cong (homography residual 0.5-1.2px, max 3.3px, dang bowl). Zhang voi model o chu nhat 25x27.5mm: fx tu 2500 -> 905..1019, rms 3->1px, |T|=5.63cm ~ do tay 5.4, R khop plan-B (tilt +5.85 roll -2.70) -> MOI PHUONG PHAP HOI TU | info: yml plan-B van dung duoc (Z neo thuoc day, khong phu thuoc sq); rms con ~1px = board cong + k1~-0.07
- [.13] pitch_from_board.py them --square-y-mm (o chu nhat); voi 27.53mm: PnP Z khop tape ca o 1.7m (truoc lech 15%) | can user do: 4 o NGANG = ? va 4 o DOC = ? (10cm hom truoc la chieu nao?)
- [.13] khuyen nghi session calib: dung PDF 9x6 25mm da in (o vuong chuan, dan bia cung phang) HOAC board tuong + do ca 2 chieu o + ep phang; script calibrate can them ho tro o chu nhat neu dung board tuong
- [.13] step F: pitch_from_board.py tao + selftest PASS (0.0000 deg, 5 goc synth) + smoke 6 cap that: pitchL 0-2 deg (xac nhan nhin thang), L-R -3.8..-5.0 (right cui hon ~4-5 deg, khop baseline nghieng), yawL -43..+26 (user doi goc moi tram), PnP Z khop tape o <1.2m (1.53 vs 1.80 o xa — frontal PnP ambiguity) | info: sweep can robot DUNG YEN, chi servo quay, cach board ~0.9-1m
- [00:10] user chot: O BOARD = 25.0mm (4 o = 10cm) — TRUNG placeholder, moi so lieu giu nguyen | info: voi sq=25 that, stereoCalibrate un-pinned cho |T|=5.92cm, fb fit 62.77/fx 1027 -> baseline hieu dung ~6.11cm; so do tay 5.4cm co ve thap hon thuc te ~10-13% -> yml nen bo pin-baseline o lan calib sau (tin scale tu square)
- [00:10] Stage E HOAN theo user; ly do: camera co SERVO cui len/xuong (pitch dong), can dung chessboard xac dinh thong so camera theo goc cui truoc khi replay
- [23:55] user bao HARDWARE CHANGE: camera nay nhin THANG (0 deg), truoc do cui xuong 30 deg | info: giai thich A/B — shots 15:06 la huong CU, captures toi nay la huong MOI -> khong tron calib moi voi anh cu; moi capture tu toi 12/07 dung stereo_rectify_20260712.yml; replay anh moi can --tilt-deg 0 (default 13 trong replay_shots.py, estimate_tilt tu sua khi co golden san); camera thang -> floor anchors yeu di, can golden/SGBM anchors tot | risk: shot_0/1/2 + session_20260712 PLY la san pham huong cu — Stage E replay can chup walk shots MOI

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
