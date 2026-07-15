# Deepmap — Free-Movement Multi-Frame Merge (Design)

Fixes the wrong merge (ghosts / rotated pieces / scale mismatch / mess) when the
robot moves freely (forward `F`, strafe `S`, turn `T`) between captures.

Three pillars:
1. **One scale for all frames** — stereo ruler stays, but tilt = run-median,
   `cam_h` corrects absolute scale, VO scale-ratio chains frames to frame 0.
2. **Measured pose, not commanded pose** — burst-capture 2D yaw during turns
   (depth-free), VO 3-DOF (yaw, tx, tz) between stops, gated per motion type.
3. **No bad frame enters the map** — QC gate before merge; ICP only refines
   already-good poses.

---

## 1. Sequence diagram — one route step

```mermaid
sequenceDiagram
    participant CO as Coordinator<br/>(route F/S/T)
    participant MW as MotionWorker<br/>(UART + Pose)
    participant TY as TurnYawTracker<br/>(turn_yaw.py, NEW)
    participant CA as CaptureWorker<br/>(stereo cams)
    participant SW as StereoWorker<br/>(golden pts)
    participant DW as DepthWorker<br/>(DA-V2)
    participant FM as FusionMapper<br/>(scale+QC+VO+merge)

    Note over CO: next step from --route,<br/>append to motions.jsonl

    alt step = F (forward) or S (strafe)
        CO->>MW: move forward / left / right d cm
        MW->>MW: Pose.advance / Pose.strafe (prior)
        MW-->>CO: move_done (K ack)
    else step = T (turn ±deg)
        CO->>TY: burst start (right cam, high rate)
        CO->>MW: spin left/right deg
        loop while turning
            TY->>TY: ORB 2D match frame i vs i-1<br/>yaw_i = atan((u_prev-cx)/fx) - atan((u_cur-cx)/fx)<br/>(depth-FREE, sign = left/right)
        end
        MW-->>CO: move_done (K ack)
        CO->>TY: burst stop
        TY-->>CO: yaw_meas = SUM yaw_i
        CO->>MW: Pose.turn(yaw_meas)  [fallback: commanded]
    end

    Note over CA: robot STATIONARY<br/>(USB cams not synced -> never stereo while moving)
    CO->>CA: capture stereo pair 1280x720
    CA->>SW: rectify -> golden points
    CA->>DW: right img -> DA-V2 relative depth
    SW-->>FM: golden (disparity)
    DW-->>FM: mono inverse-depth

    FM->>FM: fuse_metric(tilt_override = run MEDIAN tilt,<br/>floor anchors) -> Z metres + info

    alt QC FAIL (walk_qc.py NEW):<br/>inliers<8 | cam_h off>15% | tilt off>3°
        FM-->>CO: frame SKIPPED -> qc.csv row<br/>(pose still advances by prior)
    else QC pass
        FM->>FM: backproject -> level_to_floor -> cam_h
        opt cam_h off 5..15%
            FM->>FM: Z *= measured_h/cam_h -> redo project+level<br/>(absolute scale correction)
        end
        FM->>FM: VOTracker: ORB 3D vs prev stop<br/>planar 3-DOF (yaw, tx, tz) + scale ratio
        FM->>FM: gate the DOF matching step type:<br/>F: tz~cmd | S: tx~cmd | T: yaw~yaw_meas
        alt VO accepted
            FM->>FM: pose = VO pose<br/>(scale-ratio fix if 15..35% off)
        else VO rejected
            FM->>FM: pose = prior (relative dead-reckon step)
        end
        FM->>FM: icp_refine_to_map(init=pose, corr~5cm)<br/>apply ONLY if fitness good
        FM->>FM: merge: voxel nearest-wins dedup
        FM-->>CO: stop_N.ply + qc.csv row
    end

    Note over CO,FM: route done -> walk_map.ply + qc.csv + motions.jsonl<br/>(motions.jsonl = offline replay of the whole run)
```

---

## 2. Activity diagram — per-frame decision logic

```mermaid
flowchart TD
    A([next route step]) --> B{step type?}
    B -- "F / S" --> C["drive forward / strafe<br/>Pose prior += commanded"]
    B -- "T ±deg" --> D["spin + burst capture right cam"]
    D --> E["2D yaw per frame pair (no depth)<br/>total yaw_meas, sign = direction"]
    E --> F{"|yaw_meas − cmd| ok?"}
    F -- yes --> G["Pose.turn(yaw_meas)"]
    F -- "no / too few matches" --> H["Pose.turn(commanded)"]
    C --> I["capture stereo pair (stationary)"]
    G --> I
    H --> I

    I --> J["golden points + DA-V2 mono"]
    J --> K["fuse_metric<br/>tilt = run-median (servo fixed per run)<br/>floor anchors -> Z metres"]
    K --> L{"QC gate<br/>inliers ≥ 8?<br/>tilt within 3° of median?"}
    L -- fail --> M["SKIP frame -> qc.csv<br/>map untouched, walk continues"]
    M --> A

    L -- pass --> N["backproject -> level_to_floor -> cam_h"]
    N --> O{"cam_h vs measured?"}
    O -- ">15% off" --> M
    O -- "5–15% off" --> P["Z *= measured/cam_h<br/>redo project + level"]
    O -- "<5%" --> Q
    P --> Q["VO: ORB 3D match vs prev stop<br/>planar fit -> yaw, tx, tz, scale ratio"]

    Q --> R{"gate DOF of step type<br/>F: tz≈cmd | S: tx≈cmd | T: yaw≈yaw_meas<br/>rms ≤ 4cm, inliers ≥ 12"}
    R -- accept --> S{"scale ratio?"}
    S -- "within 15%" --> T["pose = VO pose"]
    S -- "15–35% off" --> U["rescale frame by ratio, refit<br/>pose = VO pose"]
    S -- ">35% off" --> V["pose = prior + relative commanded step"]
    R -- reject --> V

    T --> W["ICP refine vs accumulated map<br/>init = pose, corr ≈ 5 cm"]
    U --> W
    V --> W
    W --> X{"ICP fitness good?"}
    X -- yes --> Y["apply ICP correction"]
    X -- no --> Z["keep pose unchanged"]
    Y --> AA["merge: voxel nearest-wins dedup<br/>incremental save"]
    Z --> AA
    AA --> A

    A -- "route finished" --> AB([final walk_map.ply + qc.csv + motions.jsonl])
```

---

## 3. Logic / component diagram — modules and data flow

```mermaid
flowchart LR
    subgraph input [Inputs]
        RT["--route F30,T+30,S+20,..."]
        CAL["stereo_rectify.yml<br/>fx=1124, fB=60.69"]
        CAMH["measured cam height"]
    end

    subgraph motion [Motion layer]
        CO["Coordinator<br/>stereo_walk_map.py"]
        MW["MotionWorker<br/>robot_drive: move/spin/strafe<br/>Pose prior (F/S/T)"]
        TY["turn_yaw.py NEW<br/>burst 2D yaw tracker"]
    end

    subgraph percept [Perception layer]
        CA["CaptureWorker<br/>stereo pair @ stop"]
        SW["StereoWorker<br/>golden points"]
        DW["DepthWorker<br/>DA-V2 mono"]
        FUSE["stereo_ruler.fuse_metric<br/>+ tilt_override (run median)"]
    end

    subgraph fusion [Fusion layer — FusionMapper]
        QC["walk_qc.py NEW<br/>gate: inliers / cam_h / tilt"]
        LV["level_to_floor<br/>+ cam_h scale fix"]
        VO["visual_odom.VOTracker<br/>3-DOF gate per step type<br/>+ scale-ratio correction"]
        ICP["explore_map.icp_refine_to_map NEW<br/>pairwise, init = VO pose"]
        MG["voxel nearest-wins dedup"]
    end

    subgraph out [Outputs]
        PLY["walk_map.ply + stop_N.ply"]
        QCF["qc.csv"]
        MJ["motions.jsonl<br/>(replay input)"]
    end

    RT --> CO
    CO --> MW
    CO --> TY
    TY -- "yaw_meas" --> MW
    CO --> CA
    MW -- "pose prior" --> VO
    CAL --> SW & FUSE & VO
    CAMH --> FUSE & QC & LV
    CA --> SW & DW
    SW -- golden --> FUSE
    DW -- mono --> FUSE
    FUSE -- "Z metres + info" --> QC
    QC -- pass --> LV
    QC -- "fail: skip" --> QCF
    LV --> VO
    VO --> ICP
    ICP --> MG
    MG --> PLY
    CO --> MJ
    MJ -. "offline --replay" .-> CO
```

---

## Pose decision — 3 layers (why the map stops ghosting)

| Layer | Source | Corrects | When it loses |
|---|---|---|---|
| 1. Prior | commanded steps (dead-reckon F/S/T) | nothing — just a starting guess | always kept as fallback |
| 2. Burst yaw | 2D matches during turn (depth-free) | turn angle + direction | too few features while spinning |
| 3. VO 3-DOF | 3D matches between stops | slip on all of tx/tz/yaw + scale drift | low overlap / textureless scene |
| 4. ICP | cloud-vs-map geometry | last few cm of residual | never trusted unless fitness is good |

Each layer only refines the previous one; a failed layer falls back, never
poisons. Bad frames are removed by the QC gate *before* pose even matters.

## New / modified files

| File | Change |
|---|---|
| `deepmap/walk_qc.py` | NEW — QC record + gate + qc.csv |
| `deepmap/turn_yaw.py` | NEW — burst 2D yaw tracker for turns |
| `deepmap/stereo_walk_map.py` | route F/S/T, motions.jsonl, replay-from-log, QC + cam_h fix + ICP wiring |
| `deepmap/visual_odom.py` | motion-type-aware gates, scale-ratio correction output |
| `deepmap/explore_map.py` | `Pose.strafe`, `icp_refine_to_map`, nearest-wins voxel dedup |
| `depth-anything/src/stereo_ruler.py` | `tilt_override` param in `fuse_metric` |
