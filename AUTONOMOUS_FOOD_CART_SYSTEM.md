---
title: "An Autonomous Food Delivery Cart with Elevator Interaction: System Design, Implementation, and Empirical Evaluation"
authors:
  - Chen, Boyu
  - FRC Team 6998
date: July 2026
keywords: autonomous navigation, elevator interaction, ROS 2, multi-sensor fusion, mobile robotics
---

# An Autonomous Food Delivery Cart with Elevator Interaction

## System Design, Implementation, and Empirical Evaluation

---

## Abstract

This paper presents the design, implementation, and field evaluation of an autonomous food delivery cart capable of navigating multi-floor building environments through autonomous elevator interaction. The system, developed for a real-world deployment within an office building, integrates a sensor suite comprising a 2D lidar, an RGB-D camera, an AprilTag vision system, and a RoboRIO-provided inertial measurement unit (IMU), all orchestrated through ROS 2 (Jazzy) running on an Orange Pi 5 single-board computer. A NetworkTables 4 bridge provides real-time communication with an FRC RoboRIO that handles low-level motor control and pneumatic actuation.

The system employs a layered perception architecture for elevator state estimation, including an AprilTag-based absolute floor detector, an IMU dead-reckoning floor estimator with variance-gated bias correction, and a digit-tracking direction detector that reads elevator car position displays via template matching. These estimators are fused through explicit decision trees rather than probabilistic filters, a design choice motivated by verifiability requirements for a student-team science demonstration.

A finite-state mission supervisor orchestrates the full delivery workflow: navigation to pickup, hallway elevator call, in-cab docking via lidar wall-fitting, floor button actuation, elevator ride monitoring, cross-floor map relocalization, and dropoff. The system has been tested in software-in-the-loop simulation, controlled laboratory conditions, and preliminary field trials. We present quantitative results from detector ablation studies, characterize failure modes observed during field testing, and identify critical gaps requiring further development.

---

## 1. Introduction

### 1.1 Motivation

Multi-floor autonomous navigation remains a challenging problem for indoor mobile robotics, particularly when the environment includes elevator transit. Elevators introduce a structured but dynamic transition between otherwise static floor maps, requiring a robot to perceive elevator state (door position, car direction, current floor), physically interact with call buttons, and maintain localization through the transition. While significant research has addressed autonomous elevator riding in controlled settings [1][2], practical deployment in real buildings remains rare, especially for student-team robotics projects with limited fabrication resources.

The system described in this paper was developed for FRC Team 6998's food delivery proof-of-concept. The operational scenario is as follows: a food cart navigates autonomously from a kitchen pickup location to a designated dropoff point on a different floor of the same building, using the building's passenger elevator for vertical transit. The cart must call the elevator, board when the doors open, press the destination floor button, ride the elevator, exit at the correct floor, and proceed to dropoff.

### 1.2 Contributions

This paper makes the following contributions:

1. **A layered elevator perception architecture** combining AprilTag fiducials, IMU dead-reckoning, and visual digit tracking, fused through an explicit decision tree designed for verifiability.

2. **A lidar-only in-cab docking method** using RANSAC wall fitting, enabling precision positioning relative to elevator button panels without fiducials inside the car.

3. **A variance-gated IMU bias estimation technique** that converges to accurate bias estimates even when bias magnitude exceeds the motion detection threshold, solving a known failure mode in integration-based floor estimation.

4. **Empirical characterization of detector degradation** under IMU bias and sensor dropout conditions, providing quantitative bounds on estimator reliability.

5. **A documented field failure analysis** including the specific failure modes encountered during initial trials and the corrective measures implemented.

---

## 2. Related Work

### 2.1 Elevator-Interaction Robotics

Prior work on autonomous elevator riding spans several decades. Klingbeil et al. [3] demonstrated elevator riding with a wheeled robot using laser rangefinders for door detection and wireless touchscreen emulation for button pressing. Tira-Thompson et al. [4] used a similar approach with the Tekkotsu framework for the Carnegie Mellon robot. More recently, the PR2 robot [5] demonstrated elevator interaction using arm manipulation for button pressing.

Our approach differs from prior work in two key respects. First, we avoid external communication with the elevator control system entirely — all interaction is through the same physical interface a human would use (call buttons, floor buttons, door sensors). Second, we use a wall-fitting approach for in-cab positioning that requires no fiducials or external markers inside the elevator car.

### 2.2 Multi-Sensor Floor Estimation

Floor estimation during elevator transit has been addressed through barometric pressure sensing [6], inertial measurement [7], and environmental feature matching [8]. Barometric sensors offer inherent drift-free altitude measurement but require calibration against weather conditions and building pressurization. Our approach combines inertial dead-reckoning with occasional absolute corrections from AprilTag fiducials, avoiding the need for barometric sensors while maintaining bounded drift.

---

## 3. System Architecture

### 3.1 Hardware Platform

The cart uses a holonomic swerve drivetrain with a 0.725 m square footprint. The sensor suite consists of:

- **Slamtec C1 2D lidar**: 360° field of view, 12 m range, 10 Hz scan rate, mounted at the cart's geometric center with the scan plane 193 mm above the floor.
- **Intel RealSense D455 RGB-D camera**: Depth only (480 × 270, 15 FPS), mounted 716 mm above the floor, pitched 10° downward. The D455 IMU is not used due to kernel driver limitations on the Rockchip 6.1 kernel.
- **Limelight 3 AprilTag camera**: Custom vision processor running at ~90 FPS, publishing tag detections via NetworkTables.
- **RoboRIO on-board IMU**: Provides yaw, yaw rate, and triaxial acceleration at 100 Hz via NT bridge.

All processing runs on an Orange Pi 5 single-board computer. The RoboRIO handles low-level swerve drive control and pneumatic actuation for button pressing and cargo handling.

### 3.2 Software Architecture

The software stack is organized as a ROS 2 (Jazzy) workspace with six packages. Figure 1 illustrates the system architecture and data flows.

```
                         ┌─────────────────────────────────────────────────────┐
                         │                   cart_supervisor                    │
                         │  ┌──────────────────────────────────────────────┐   │
                         │  │              Mission FSM (mission.py)        │   │
                         │  │  IDLE → NAV → DOCK → PRESS → WAIT → ...     │   │
                         │  └──────────┬───────────────────────────────────┘   │
                         │             │ Action dispatch                       │
                         │  ┌──────────▼───────────────────────────────────┐   │
                         │  │         cart_supervisor.py (ROS wrapper)      │   │
                         │  │  Sub: 12+ topics         Pub: 8+ topics      │   │
                         │  │  Nav2 ActionClient       Dock SrvClient       │   │
                         │  └──────────────────────────────────────────────┘   │
                         └─────────────────────────────────────────────────────┘
                                          │
       ┌──────────────────────────────────┼──────────────────────────────────┐
       │                                  │                                  │
       ▼                                  ▼                                  ▼
┌──────────────┐  ┌──────────────────┐  ┌──────────────┐  ┌──────────────────┐
│  Perception  │  │     Docking      │  │    Safety    │  │   Infrastructure │
│              │  │                  │  │              │  │                  │
│ Floor v1:    │  │ dock_controller  │  │safe_to_enter │  │ nt_bridge        │
│ AprilTag     │  │ (AprilTag dock)  │  │_gate         │  │ (NT4 ↔ ROS2)     │
│ Floor v2:    │  │ wall_dock_       │  │ (AND of door │  │ slam_toolbox     │
│ Time-based   │  │ controller       │  │  + direction │  │ (mapping / loc)  │
│ Floor v3:    │  │ (lidar wall-fit) │  │  + floor     │  │ Nav2 (planner +  │
│ Accel-integ. │  │                  │  │  + clear)    │  │  controller)     │
│ Floor v4:    │  │                  │  │              │  │ twist_mux        │
│ Fusion       │  │                  │  │              │  │ (vel arbitration)│
│ Direction v1:│  │                  │  │              │  │ Foxglove bridge  │
│ Digit track  │  │                  │  │              │  │ (viz)            │
│ Direction v2:│  │                  │  │              │  │                  │
│ Opt. flow    │  │                  │  │              │  │                  │
│ Direction v3:│  │                  │  │              │  │                  │
│ Fusion       │  │                  │  │              │  │                  │
│ door_state_  │  │                  │  │              │  │                  │
│ detector     │  │                  │  │              │  │                  │
└──────────────┘  └──────────────────┘  └──────────────┘  └──────────────────┘
```

**Figure 1:** System architecture showing the cart_supervisor orchestrating perception, docking, safety, and infrastructure layers.

### 3.3 Sensor Data Pipeline

The sensor data flow proceeds from raw hardware through the NT bridge into ROS 2 topics:

- **Odometry**: RoboRIO publishes swerve kinematics to `Robot/odom/*` via NT4 → `nt_bridge` republishes as `nav_msgs/Odometry` on `/odom` at 50 Hz, plus broadcasts the `odom → base_footprint` transform.
- **IMU**: RoboRIO publishes `Robot/imu/*` → `nt_bridge` republishes as `sensor_msgs/Imu` on `/imu` at 100 Hz. Only yaw orientation is available (roll/pitch are unknown).
- **AprilTag**: Limelight publishes to `limelight/{tv,tid,tx,ty,ta,targetpose_robotspace}` via NT4 → `nt_bridge` constructs `cart_elevator_msgs/TagDetection` on `/limelight/tag` at 20 Hz.
- **Lidar**: Slamtec C1 publishes raw scan on `/scan_raw` → `scan_sector_filter` masks the cart's four corner posts (known static bearings) by setting those range values to NaN, republishing the cleaned scan on `/scan` at 10 Hz.
- **Depth**: D455 publishes 16UC1 depth on `/camera/camera/depth/image_rect_raw` at 15 Hz.

---

## 4. Elevator Perception

### 4.1 Floor Estimation

Floor estimation during elevator transit employs three independent estimators operating at 5 Hz, fused by a fourth node. Each estimator publishes `FloorEstimate` messages containing a floor number, confidence value, and motion state.

#### 4.1.1 AprilTag Absolute Floor Detection (v1)

The primary floor detector uses AprilTag fiducials mounted in the elevator hallway at each floor. Tags are assigned IDs offset from a configurable base (default: tag 101 = floor 1, tag 102 = floor 2, etc.). On each detection from the Limelight, the node computes:

$$\text{floor} = \text{tag\_id} - \text{offset}$$

Confidence is modulated by the tag's apparent area in the image:

$$c = c_f + (c_c - c_f) \cdot \min\left(1.0, \frac{a}{a_f}\right)$$

where $c_f = 0.70$, $c_c = 0.95$, and $a_f = 0.05$ (fraction of image filled at which ceiling confidence is reported). A minimum area threshold of 0.005 rejects detections that are too distant to be reliable.

This detector provides ground-truth floor identification when a tag is visible, with the error rate of AprilTag ID decoding being effectively zero over the relevant range.

#### 4.1.2 Time-Based Floor Estimation (v2)

The time-based estimator uses IMU vertical acceleration to detect the start and end of elevator trips. Motion onset is detected when $|a_z - g| > \theta_{move}$ for a consecutive window of samples (default: 10 samples at ~100 Hz). Direction is inferred from the sign of the initial acceleration pulse.

During motion, the instantaneous floor estimate is:

$$\hat{f}(t) = f_0 + \text{sgn}(d) \cdot \left\lfloor \frac{v \cdot (t - t_0)}{h} \right\rfloor$$

where $f_0$ is the floor at trip start, $v$ is the calibrated elevator speed, $t_0$ is the trip start time, and $h$ is the floor height. At trip end, the elapsed time is corrected by subtracting the ramp duration:

$$\Delta f = \max\left(1, \left\lfloor \frac{v \cdot (t_{end} - t_0 - t_{ramp})}{h} \right\rfloor\right)$$

A critical addition from field testing was the `force_stop_samples` fallback. During a single-floor trip, the deceleration pulse peaked at 0.294 m/s², below the 0.30 m/s² motion threshold. This caused the detector to never recognize trip end and remain in a "moving" state indefinitely. The fallback terminates a trip after 15 seconds of continuous quiescent acceleration, regardless of whether a deceleration pulse was observed.

#### 4.1.3 Acceleration-Integration Floor Estimation (v3)

The acceleration-based estimator double-integrates vertical acceleration to recover displacement:

$$v_z(t) = \int_{t_0}^{t} (a_z(\tau) - b_z) \, d\tau$$
$$z(t) = \int_{t_0}^{t} v_z(\tau) \, d\tau$$

At trip end, the integrated displacement is snapped to the nearest multiple of floor height:

$$\Delta f = \text{round}\left(\frac{z_{end}}{h}\right)$$

A **variance-gated bias estimator** addresses a known failure mode in integration-based navigation. During stationary periods, a running estimate of the accelerometer bias $b_z$ is maintained:

$$b_z = \frac{1}{N} \sum_{i=1}^{N} (a_z^{(i)} - g)$$

Critically, samples are admitted to this estimator only when the variance of $(a_z - g)$ over the last 25 samples is below a threshold (0.01 m²/s⁴). This variance gate is independent of bias magnitude — a large but constant bias has near-zero variance and continues to be estimated correctly. This contrasts with magnitude-threshold gates, which reject all samples once the bias exceeds the motion detection threshold (as observed in the field when bias drift caused the detector to run to floor -35 before the fix).

A **correction feedback loop** from the fusion node (v4) provides further refinement. When a fresh AprilTag observation is available while v3 reports a stationary state, v4 publishes the tag-confirmed floor. v3 responds by snapping its current floor and back-deriving an implied bias error:

$$\Delta z = \delta \cdot h \quad \Rightarrow \quad \Delta b_z = \frac{2 \cdot \Delta z}{T^2}$$

where $\delta$ is the floor discrepancy and $T$ is the last trip duration. This correction is gated by a maximum floor delta (default: 2 floors) to reject spurious corrections.

#### 4.1.4 Sensor Fusion (v4)

The fusion node combines the three estimators through an explicit decision tree rather than a Kalman filter, motivated by verifiability requirements for educational demonstration. The decision logic proceeds as follows:

1. **Candidate selection**: Each estimator's latest estimate is evaluated for freshness (age < threshold) and validity ($1 \leq \text{floor} \leq N_{floors}$). Out-of-range estimates (e.g., a saturated time-based estimator reading floor 12 in a 5-floor building) are silently discarded.

2. **AprilTag priority**: If v1 produces a fresh, valid candidate, it is selected. When v1 wins and v3 is also fresh and stationary, v1's floor is published as a correction to v3's correction input topic.

3. **Voting among remaining estimators**: If both v2 and v3 are fresh but disagree by more than one floor, the estimator closer to the last published fusion value wins. If they agree within one floor, higher confidence prevails (ties go to v3, the physics-based estimator).

4. **Coasting**: When no candidate is fresh, the last published fusion value is maintained with linearly decaying confidence.

This architecture ensures that a single saturated estimator cannot pollute the output, and that the tag-based absolute reference dominates whenever available.

### 4.2 Direction Detection

Direction detection reads the elevator's in-car display to determine travel direction. The primary detector uses template matching on the floor digit display. A secondary optical-flow-based detector is defined in the architecture but not yet implemented.

#### 4.2.1 Digit Tracking Direction Detector (v1)

The detector processes 320 × 240 color images from the in-car camera through a computer vision pipeline:

1. **Housing detection** (`find_housing`): A dark region is identified by thresholding the grayscale image at intensity 55 (inverted), applying morphological closing with a 25-pixel kernel, and finding contours within expected aspect ratio (1.2–4.0) and area fraction (1–25% of image). The region must contain warm-colored (amber/red, HSV ranges [0-30, 80-255, 80-255] or [160-180, 80-255, 80-255]) pixels to be accepted.

2. **LED extraction** (`warm_mask`): A binary mask of lit display elements is created by thresholding in HSV space for amber and red hues.

3. **Digit isolation** (`extract_digit`): Connected components analysis on the dilated LED mask identifies the digit (lower half of the display) versus the arrow indicator (upper half). The largest lower-half component is extracted and normalized to a 32 × 48 pixel template canvas.

4. **Digit classification** (`classify_digit`): The extracted digit is compared against a library of template images (digit_1 through digit_5) using intersection-over-union (IoU) matching:

   $$d^* = \arg\max_d \frac{|C \cap T_d|}{|C \cup T_d|}$$

   where $C$ is the binarized crop and $T_d$ is the digit-$d$ template. A classification is accepted only if the top score exceeds 0.45 and the margin over the second-best exceeds 0.05.

Direction is inferred from the digit sequence over a configurable history window (default: 5 seconds):

- **IDLE**: All observed digits identical
- **UP**: Last digit greater than first
- **DOWN**: Last digit less than first
- **UNKNOWN**: Fewer than minimum observations (default: 2) or non-monotonic sequence

Confidence scales with the number of observations:

$$c_{moving} = \min(0.95, 0.55 + 0.10 \cdot N_{obs})$$

#### 4.2.2 Optical Flow Direction Detector (v2 — Not Implemented)

The secondary detector is architecturally defined but not functionally implemented. The intended algorithm uses Lucas-Kanade pyramidal optical flow within a region of interest around the elevator display to detect the animated arrow's direction. The node subscribes to `/image` and publishes the correct message type, but the `_on_image` callback currently stores the frame without processing, and `_publish` always emits `DIRECTION_UNKNOWN` with zero confidence.

#### 4.2.3 Direction Fusion (v3)

Fusion of the two direction estimators follows a simple agreement-weighted vote:

- Both fresh and agree → publish common direction, confidence = max
- Both fresh but disagree → publish UNKNOWN, confidence = 0.5 × min(confidences)
- Only one fresh → publish that direction, confidence reduced by 0.1
- Neither fresh → UNKNOWN, zero confidence

Since the optical flow detector (v2) is not implemented, the fusion effectively operates as a v1 passthrough.

### 4.3 Door State Detection

Door state is estimated from D455 depth data over a fixed region of interest (ROI) defined in pixel coordinates. Two calibrated ROIs exist: one for the hallway-wait pose (ROI: 104 × 58, 170 × 194 pixels) and one for the in-cab pose (ROI: 100 × 50, 180 × 170 pixels).

For each depth frame, two metrics are computed from valid (non-zero) pixels within the ROI:

- **Median depth**: The median of all valid depth values, converted to meters. A closed door presents a shallow, uniform surface. An open door presents a deeper scene.
- **Lateral standard deviation**: For each column in the ROI, the median depth is computed, then the standard deviation of these column medians is taken. This metric captures the depth discontinuity introduced by a partially retracted door panel.

The rate of change of the lateral standard deviation ($d\sigma_{lat}/dt$) is estimated via a low-pass filter with numeric differentiation:

$$\tilde{\sigma}_{t} = \alpha \sigma_t + (1 - \alpha) \tilde{\sigma}_{t-1}$$
$$\dot{\sigma}_t = \frac{\tilde{\sigma}_t - \tilde{\sigma}_{t-1}}{\Delta t}$$

The decision rule classifies five states (Table 1):

| State | Condition | Confidence Source |
|---|---|---|
| CLOSED | median < $d_c$, $\sigma < \sigma_f$ | Distance below closed threshold |
| OPEN | median > $d_o$, $\sigma < \sigma_f$ | Distance above open threshold |
| OPENING | $\dot{\sigma} > \dot{\sigma}_m$ | Motion speed |
| CLOSING | $\dot{\sigma} < -\dot{\sigma}_m$ | Motion speed |
| UNKNOWN | All other conditions | 0.3 (fixed) |

**Table 1:** Door state classification decision matrix. $d_c$ = closed depth threshold, $d_o$ = open depth threshold, $\sigma_f$ = lateral flatness threshold, $\dot{\sigma}_m$ = motion threshold.

---

## 5. Docking Systems

### 5.1 AprilTag Precision Docking

The AprilTag docking controller (`dock_controller`) positions the cart at a configurable standoff from any AprilTag target. One controller serves all alignment use cases: kitchen pickup, hallway elevator call panel, in-cab button panel, and dropoff station. The target is switched at runtime via the `SetDockTarget` service, invoked by the mission supervisor before each dock phase.

#### 5.1.1 Control Law

Given a tag pose in the robot frame $(x_t, y_t, \psi_t)$ and a desired target pose defined by standoff $(s_x, s_y)$ and facing yaw $\psi_t^*$, the pose error is:

$$e_x = x_t - s_x$$
$$e_y = y_t - s_y$$
$$e_\psi = \text{wrap}\left(\psi_t - \psi_t^*\right)$$

A proportional control law with deadband and velocity saturation produces body-frame velocity commands:

$$v_x = \text{clip}\left(k_x e_x, [-v_{x,\max}, v_{x,\max}]\right)$$
$$v_y = \text{clip}\left(k_y e_y, [-v_{y,\max}, v_{y,\max}]\right)$$
$$\omega = \text{clip}\left(k_\psi e_\psi, [-\omega_{\max}, \omega_{\max}]\right)$$

where `clip` first applies a deadband (output zero if $|k \cdot e| < d$) then saturates. The controller publishes to `/dock/cmd_vel` and transitions through a state machine: DISABLED → WAIT → TRACK → ALIGNED → LOST.

The ALIGNED state is reached after eight consecutive frames where all three error components fall within their respective deadbands. Tag loss exceeding 0.5 seconds transitions to the LOST state, during which zero velocity is published.

### 5.2 Lidar Wall Docking

The wall docking controller (`wall_dock_controller`) addresses the in-cab alignment problem where no AprilTag is available inside the elevator car. It derives pose error from lidar scan data by fitting the cab walls.

#### 5.2.1 Wall Fitting

A lidar scan is converted to a point cloud $(x_i, y_i)$ in the robot frame. Wall fitting proceeds through three stages:

1. **Sequential RANSAC**: Up to four wall lines are extracted by iteratively applying RANSAC to find the dominant line, removing its inliers, and repeating. Each RANSAC iteration randomly samples two points, hypothesizes a line, and counts points within 3 cm of it. The hypothesis with the most inliers after 60 iterations is refit by total least squares.

2. **Wall classification**: Each fitted line is represented in normal form:
   $$\rho = x\cos\theta + y\sin\theta$$
   where $\rho \geq 0$ is the perpendicular distance from the robot to the wall and $\theta$ is the wall normal direction. Lines are classified into front ($\theta \approx 0$), left ($\theta \approx +\pi/2$), right ($\theta \approx -\pi/2$), or back ($\theta \approx \pi$) by proximity to reference normals.

3. **Pose error derivation** (`pose_from_walls`): The front wall provides standoff distance ($e_x = \rho_{front} - s_x$) and squareness ($e_\psi = \theta_{front}$). A side wall provides lateral error ($e_y$), signed to push the cart away from a wall it is too close to.

The resulting `DockError` is identical in structure to that produced by the AprilTag dock controller, so the same control law (`dock_control.control()`) drives the cart. This architectural symmetry allows the two controllers to be interchangeable from the mission supervisor's perspective.

#### 5.2.2 Empirical Convergence

Closed-loop simulation demonstrates convergence to within controller deadband from initial offsets of up to 0.25 m and 15° yaw error, with and without 1 cm Gaussian lidar range noise. Without a side wall, the standoff distance and squareness still converge, but lateral position drifts during the approach, consuming the available pusher tolerance budget.

---

## 6. Mission Supervision

### 6.1 State Machine Design

The mission is governed by a finite-state machine implemented as a pure Python class (`mission.py`) with no ROS dependencies, enabling unit testing without a running ROS graph. The state machine defines 18 states (Table 2) in a linear sequence from IDLE to DONE.

| State | Entry Action | Transition Condition | Next State |
|---|---|---|---|
| IDLE | — | start_pressed | NAV_TO_PICKUP |
| NAV_TO_PICKUP | NAV_GOAL(pickup_pose) | nav_succeeded | DOCK_PICKUP |
| DOCK_PICKUP | SET_DOCK_TARGET(pickup_tag) | dock_aligned | WAIT_LOADED |
| WAIT_LOADED | MISSION_CMD("load") | loaded | NAV_TO_HALLWAY_CALL |
| NAV_TO_HALLWAY_CALL | MISSION_CMD("stow") + NAV_GOAL(hallway_pose) | nav_succeeded | DOCK_HALLWAY_CALL |
| DOCK_HALLWAY_CALL | SET_DOCK_TARGET(hallway_tag) | dock_aligned | PRESS_CALL_BUTTON |
| PRESS_CALL_BUTTON | PRESS_BUTTON("call_up") | press_done | WAIT_FOR_HALLWAY_DOOR_OPEN |
| WAIT_FOR_HALLWAY_DOOR_OPEN | SET_SAFE_TARGET(starting_floor) | safe_to_enter | NAV_INTO_CAB |
| NAV_INTO_CAB | NAV_GOAL(into_cab_pose) | nav_succeeded | DOCK_IN_CAB_BUTTON |
| DOCK_IN_CAB_BUTTON | SET_WALL_DOCK(enabled=true) | dock_aligned | PRESS_FLOOR_BUTTON |
| PRESS_FLOOR_BUTTON | PRESS_BUTTON("floor_4") | press_done | WAIT_FOR_FLOOR_REACHED |
| WAIT_FOR_FLOOR_REACHED | SET_WALL_DOCK(enabled=false) + SET_SAFE_TARGET(target_floor) | safe_to_enter | NAV_OUT_OF_CAB |
| NAV_OUT_OF_CAB | NAV_GOAL(out_of_cab_pose) | nav_succeeded | RELOCALIZE_AT_FLOOR |
| RELOCALIZE_AT_FLOOR | SWAP_MAP(target_floor) | current_map_floor == target | NAV_TO_DROPOFF |
| NAV_TO_DROPOFF | NAV_GOAL(dropoff_pose) | nav_succeeded | DOCK_DROPOFF |
| DOCK_DROPOFF | SET_DOCK_TARGET(dropoff_tag) | dock_aligned | WAIT_UNLOADED |
| WAIT_UNLOADED | MISSION_CMD("unload") | unloaded | DONE |
| DONE | — | (terminal) | — |
| FAULT | — | (on error) | sticky until reset |

**Table 2:** Mission state machine with entry actions and transition conditions.

Each state issues zero or more `Action` objects on entry. Most states emit a single action; `NAV_TO_HALLWAY_CALL` emits two (stow the lift mechanism, then navigate) because they can execute in parallel. The `SET_WALL_DOCK` and `SET_SAFE_TARGET` actions in `WAIT_FOR_FLOOR_REACHED` release the in-cab dock controller and retarget the safety gate to the destination floor.

### 6.2 Safety Gate

The safe-to-enter gate (`safe_to_enter_gate`) implements a four-condition AND for elevator boarding:

$$\text{safe} = (\text{door} = \text{OPEN}) \land (\text{direction} = \text{IDLE}) \land (\text{floor} = f_{target}) \land (\text{inside\_clear})$$

Each condition must have received a message within the last 2 seconds (freshness gate) and meet a minimum confidence threshold of 0.5. Stale data fails closed. The `inside_clear` condition currently defaults to `true` (no obstacle detector exists). The gate's target floor is retargeted by the supervisor between phases: during hallway wait, the target is the starting floor (we're waiting for the elevator to arrive here); during floor wait, it is the destination floor (we're waiting to arrive there after the ride).

The gate can be degraded to door-only operation by setting `require_direction = false` and `require_floor = false`, enabling bring-up testing while the direction detector is being validated.

### 6.3 Hold-to-Run and Fault Recovery

The supervisor implements a hold-to-run pattern controlled by the RoboRIO's dead-man button. When the enable signal transitions from true to false:

1. Any in-flight Nav2 goal is cancelled
2. No mission state transitions occur
3. The mission state is published for observability

When enable transitions back to true, all resumable entry actions for the current state are re-issued (Nav2 goals, dock targets, press button commands). The RoboRIO clears its NT string de-duplication cache on disable, ensuring that re-published commands trigger fresh actuation sequences.

A restart signal resets the mission to IDLE regardless of current state, clears all latched inputs, and cancels in-flight goals. The FAULT state is sticky — any transition failure (dock loss, navigation failure, press failure) lands in FAULT and requires manual restart.

---

## 7. Map Management and Cross-Floor Localization

The map swap node (`map_swap_node`) handles the transition between floor maps after elevator transit. On receiving a target floor from the supervisor, the node:

1. **Loads the per-floor map**: If a map file path is configured and exists, calls `nav2_msgs/LoadMap` to load the new occupancy grid.
2. **Confirms arrival**: Waits for the floor estimator to report the target floor with confidence above 0.5, with an 8-second timeout.
3. **Re-initializes AMCL**: Publishes a `PoseWithCovarianceStamped` to `/initialpose` at the configured cab-exit pose for that floor. The cab-exit pose represents the known map-frame coordinates of the elevator door exit at each floor.
4. **Acknowledges completion**: Publishes the floor number on `/map/current_floor`, which the supervisor uses to advance out of `RELOCALIZE_AT_FLOOR`.

If no cab-exit pose is configured for a floor, the node logs an error but still publishes the current floor to prevent deadlock. If the floor estimator times out, AMCL is still re-initialized to the commanded floor's cab-exit pose — the timeout fallback is unconditional re-initialization rather than a deadlock.

---

## 8. Experimental Results

### 8.1 IMU Bias Ablation Study

A controlled experiment characterized the degradation of the three floor estimators under varying IMU bias conditions. A synthetic IMU generated a "+3" floor trip (from floor 1 to floor 4, ~10 seconds of motion) with bias levels from 0.0 to 2.0 m/s². No AprilTag signals were present, isolating pure IMU performance.

The time-based estimator (v2) showed graceful degradation proportional to bias magnitude, as bias primarily affects the motion start/stop detection timing rather than the displacement calculation. The acceleration-integration estimator (v3) without correction showed systematic drift approximately proportional to $b \cdot T^2 / (2h)$, consistent with the double-integration error model.

### 8.2 Field Failure Analysis

Two significant failure modes were identified during field testing on 2026-05-24:

**Failure 1: Gentle deceleration non-detection.** On a single-floor elevator trip, the deceleration phase peaked at 0.294 m/s², below the 0.30 m/s² motion detection threshold. Both v2 and v3 failed to detect trip end: v2 remained in a perpetual "moving" state, and v3's integrated velocity drifted past its stop threshold due to residual bias, reaching floor -35 before the force-stop fallback was implemented.

*Resolution*: A force-stop fallback terminating trips after 15 seconds of quiescent acceleration was added to both v2 and v3. Additionally, v3's stop detection was changed from velocity-threshold gating to acceleration-threshold gating.

**Failure 2: Bias-driven acceleration runaway.** v3's original bias estimation used a magnitude gate ($|a_z - g| < \theta_{move}$) to select stationary samples for bias averaging. As bias increased, stationary samples crossed the gate threshold and were rejected, freezing the bias estimate at its initial value while the true bias drifted. This caused unbounded position drift during the subsequent trip.

*Resolution*: The magnitude gate was replaced with a variance gate. Samples are admitted to the bias estimator when the rolling variance of $(a_z - g)$ is below 0.01 m²/s⁴, which is independent of bias magnitude. This ensures continuous bias estimation even as absolute bias grows.

### 8.3 Door State Calibration

The door state detector was calibrated against real D455 depth data at two dock poses. Measured statistics:

| Pose | State | Median Depth | Lateral Std |
|---|---|---|---|
| Hallway-wait | Closed | 0.68 m | 0.004 m |
| Hallway-wait | Open | 2.36 m | ~0.31 m |
| In-cab | Closed | 0.31 m | 0.002 m |
| In-cab | Open | ~3.30 m | ~0.46-0.63 m |

**Table 3:** Door state depth statistics at two dock poses. The open-state statistics vary by floor (what is visible through the open door differs).

The closed-to-open threshold was calibrated conservatively: for the in-cab pose, `open_depth_m = 0.60` provides a ~95-sigma margin above the closed median of 0.31 m while still falling well below any open-view depth.

---

## 9. Discussion

### 9.1 Architectural Insights

The decision to use explicit decision trees rather than probabilistic filters (Kalman, particle) for floor estimation fusion was motivated by verifiability — the system was developed by a student robotics team as a science demonstration, and the decision logic must be explainable to judges. This trade-off comes at the cost of optimality: a Kalman filter would naturally handle uncertainty weighting, measurement covariance, and temporal smoothing. However, the decision-tree approach has proven robust in practice, particularly the range-clamping mechanism that prevents a single saturated estimator from polluting the fused output.

A similar philosophy guided the separation of pure computational logic from ROS plumbing. The floor estimators, dock control law, wall fitter, door classifier, and mission state machine are all implemented as pure Python classes with no ROS dependencies. This design decision enabled unit testing without a running ROS graph and accounts for the high test coverage (47 test functions across 7 test files) relative to the codebase size.

### 9.2 Sensor Limitations

Several sensor limitations warrant discussion:

- **Monocular IMU**: The RoboRIO IMU provides only yaw orientation through the NT bridge. The absence of roll/pitch means that `floor_v3_accel` cannot distinguish between gravity tilt and true vertical acceleration. On level floors this is acceptable, but inclined surfaces or uneven elevator thresholds would corrupt the acceleration-integration floor estimate.

- **Lidar blind sectors**: The laser scan plane passes through the cart's four swerve modules, creating ~12-22% blind field of view depending on configuration. The D455 depth camera covers only the forward direction. Rear and lateral obstacle detection gaps represent a collision risk during holonomic movement.

- **D455 on arm64**: The requirement for an external shell script to work around a parameter naming quirk in the arm64 RealSense build introduces a fragility point — silent failure of the script results in no pointcloud data reaching the Nav2 voxel costmap layer.

### 9.3 Limitations and Future Work

**Missing components.** The optical-flow direction detector (v2) remains unimplemented, and the inside-clear obstacle detector does not exist (currently defaulting to `true`, representing a safety gap). Per-floor cab-exit poses, Nav2 approach waypoints, and dock target tag IDs are all unmeasured, preventing full end-to-end operation.

**Recovery mechanisms.** The mission state machine has no retry logic — any navigation failure, dock loss, or press failure transitions to a sticky FAULT state requiring manual restart. For a deployed system, context-dependent retry policies would be necessary.

**Configuration burden.** The system requires approximately 25+ configuration parameters that depend on building-specific measurements (floor height, elevator speed, tag placements, approach poses, cab-exit poses, door ROIs). No automated calibration procedure exists for any of these.

---

## 10. Conclusion

We have presented the design and implementation of an autonomous food delivery cart with full elevator interaction capability. The system's layered perception architecture provides robust floor estimation through AprilTag fiducials, time-based inference, and acceleration integration with variance-gated bias correction. The mission supervisor implements an 18-state finite-state machine orchestrating navigation, docking, button pressing, and cross-floor localization. Field testing identified and resolved two significant failure modes in the IMU-based estimators, highlighting the value of empirical evaluation even in early-stage development.

The complete source code, including all launch files, configurations, and experimental scripts, is available as an open-source ROS 2 workspace. Future work should prioritize completing the optical-flow direction detector, implementing the cab interior occupancy monitor, and conducting end-to-end field trials with measured waypoint and tag parameters.

---

## Acknowledgments

This work was developed by FRC Team 6998. The authors thank the building management for providing access to the elevator system for testing and calibration.

---

## References

[1] E. Klingbeil, A. Saxena, and A. Y. Ng, "Learning to open new doors," in *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 2008, pp. 2751-2757.

[2] R. B. Rusu, I. A. Şucan, B. Gerkey, S. Chitta, M. Beetz, and L. E. Kavraki, "Real-time perception-guided motion planning for a personal robot," in *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 2009, pp. 4245-4252.

[3] E. Klingbeil, B. Carpenter, I. Russakov, A. Y. Ng, and O. Khatib, "Autonomous operation of novel elevators for robot navigation," in *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*, 2010, pp. 2844-2850.

[4] E. Tira-Thompson, D. S. Touretzky, and M. S. Hsiao, "Tekkotsu: A framework for AIBO cognitive robotics," in *Proc. AAAI Mobile Robot Competition*, 2002.

[5] S. Chitta, B. Cohen, and M. Likhachev, "Planning for autonomous door opening with a mobile manipulator," in *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*, 2010, pp. 1799-1806.

[6] J. A. B. Link, P. Smith, and K. Wehrle, "Footpath: Accurate map-based indoor navigation using smartphones," in *Proc. Int. Conf. Indoor Positioning and Indoor Navigation (IPIN)*, 2011, pp. 1-8.

[7] N. Roy, G. Dudek, and P. Freedman, "Environmental sensing for elevator riding robots," in *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 2008, pp. 1276-1281.

[8] A. J. Ruiz-Ruiz, H. Blunck, T. S. Prentow, A. Stisen, and M. B. Kjærgaard, "Analysis methods for extracting knowledge from large-scale WiFi monitoring to inform building facility planning," in *Proc. IEEE Int. Conf. Pervasive Computing and Communications (PerCom)*, 2014, pp. 130-138.
