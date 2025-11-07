# Autoparking

<div align="center">
<img width="600px" src="https://github.com/GuilleAQ/Service-Robots_25-26/blob/main/P3/resources/figures/1.png">
</div>

<h3 align="center"> Autoparking </h3>

<div align="center">
<img width=140px src="https://img.shields.io/badge/status-complete-brightgreen">
<img width=140px src="https://img.shields.io/badge/language-Python-blue">
</div>

---

## Table of Contents
- [Task Description](#task-description)
- [Robot API](#robot-api)
- [1. Sensor Processing](#1-sensor-processing)
- [2. Yaw Control (PD Controller)](#2-yaw-control-pd-controller)
- [3. Parking State Machine](#3-parking-state-machine)
- [4. Gap Detection Strategy](#4-gap-detection-strategy)
- [5. Lateral Alignment and Street Following](#5-lateral-alignment-and-street-following)
- [6. Reverse Maneuver Phases](#6-reverse-maneuver-phases)
- [Video Demo](#Video-Demo)

---

## Task Description

The objective of this exercise is to implement the logic of a navigation algorithm for an automated vehicle. The vehicle must find a parking space and park properly.

requirements:

- Control the vehicle using **V (linear speed)** and **W (angular speed)**
- Detect and select a valid parking space using laser data
- Park between two cars


---

## Robot API

|  |  |
|---|---|
| `HAL.getPose3d()` | Returns the robot pose (includes `x`, `y`, `yaw`). |
| `HAL.getPose3d().x` | Robot **x** coordinate (meters). |
| `HAL.getPose3d().y` | Robot **y** coordinate (meters). |
| `HAL.getPose3d().yaw` | Robot yaw/orientation relative to world frame (radians). |
| `HAL.getFrontLaserData()` | Get **front laser** readings (180 values). |
| `HAL.getRightLaserData()` | Get **right laser** readings (180 values). |
| `HAL.getBackLaserData()` | Get **rear laser** readings (180 values). |
| `HAL.setV(v)` | Set linear velocity **V** (m/s). |
| `HAL.setW(w)` | Set angular velocity **W** (rad/s). |


---

### Laser Sensor Attributes

`HAL.getFrontLaserData()`, `HAL.getRightLaserData()`, and `HAL.getBackLaserData()` return an object with:

|  |  |
|---|---|
| `minAngle` | Starting angle of scan *(radians)* |
| `maxAngle` | Ending angle of scan *(radians)* |
| `minRange` | Minimum valid measurement *(meters)* |
| `maxRange` | Maximum valid measurement *(meters)* |
| `values` | List of **180 distance measurements** *(meters)* |

> Measurements `< minRange`, `> maxRange`, or `inf` should be discarded.

---

## 1. Sensor Processing

Laser readings are filtered to remove invalid values (`inf`, out-of-range) and extract meaningful metrics:

- **Mean distance** - detect continuous free corridor
- **Minimum distance** - detect approaching obstacle
- **Band check** - maintain desired lateral distance (≈0.6 m)

This ensures stable lateral reference from the curb/parked vehicles.

---

## 2. Yaw Control (PD Controller)

A **PD controller** provides smooth heading alignment:

```python
error = wrap_angle(target_yaw - current_yaw)
w = kp * error + kd * (error - prev_error)
```
### Used for:

- Initial street alignment  
- Angular stabilization during reverse phases

---

## 3. Parking State Machine

|  |  |
|-------|----------|
| **SEARCHING_PARKING** | Follow street, detect empty space |
| **MOVE_TO_BOX** | Align to curb + correct lateral distance |
| **PARKING** | Execute reverse maneuver in phases |
| **PARKED** | Stop, vehicle fully parked |

This sequential logic allows predictable and explainable behavior.

---

## 4. Gap Detection Strategy

We detect a parking slot by observing a **sudden increase** in right-laser average distance:

```python
if mean_right >= PARKING_GAP_THRESHOLD:
    state = MOVE_TO_BOX
```
This technique identifies a real street-side gap, not just noise.

---

## 5. Lateral Alignment and Street Following

The robot sets itself to a **target lateral offset** from the curb (≈ **0.6 m**):

- If **too close** - **steer left**
- If **too far** - **steer right**

Yaw is corrected **only when lateral distance is acceptable**, stabilizing the approach. This ensures the vehicle aligns with the street even when starting angled or offset.

---

## 6. Reverse Maneuver Phases

|  |                                    |
|------:|--------------------------------------------|
| **1** | Reverse with steering toward curb          |
| **2** | Continue reversing with softer counter-steer |
| **3** | Forward short adjustment if needed         |
| **4** | Final fine yaw correction and stop         |

**Transitions depend on:**

- Rear obstacle detection  
- Front obstacle proximity  
- Yaw tolerance thresholds  

This models how human drivers parallel-park.

---

## Video Demo
[video demo](https://urjc-my.sharepoint.com/:v:/g/personal/g_alcocer_2020_alumnos_urjc_es/EdPEcCMYNwxFjf2Lax_1kU0BRYLC_YgndFJNTPGGuxJD3A?e=EomQNG)

