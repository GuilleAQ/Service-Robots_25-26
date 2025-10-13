# Localized Vacuum Cleaner

<div align="center">
<img width="600px" src="https://github.com/GuilleAQ/Service-Robots_25-26/blob/main/P1/resources/figures/1.png">
</div>

<h3 align="center"> Localized Vacuum Cleaner — Implementation and Explanation </h3>

<div align="center">
<img width=120px src="https://img.shields.io/badge/status-complete-brightgreen" alt="status">
<img width=120px src="https://img.shields.io/badge/language-Python-blue" alt="language">
</div>

## Table of Contents
- [Task Description](#task-description)
- [Robot API (HAL / WebGUI) Used in the Practice](#robot-api-hal--webgui-used-in-the-practice)
- [1. Map Loading and Preprocessing](#1-map-loading-and-preprocessing)
- [2. Coordinate Transformations](#2-coordinate-transformations)
- [3. Grid Construction](#3-grid-construction)
- [4. Search Algorithms (BFS)](#4-search-algorithms-bfs)
- [5. Motion Control to a Grid Cell](#5-motion-control-to-a-grid-cell)
- [6. Exploration Logic and State Machine](#6-exploration-logic-and-state-machine)
- [7. Visualization and Debug in WebGUI](#7-visualization-and-debug-in-webgui)
- [Video Demo](#Video-Demo)
---

## Task Description

The objective of this exercise is to implement the logic of a navigation algorithm for an autonomous vacuum cleaner by making use of the location of the robot. 
The robot is equipped with a map and knows it’s current location in it. The main objective will be to cover the largest area of ​​a house using the programmed algorithm.

---


## Robot API (HAL / WebGUI) Used in the Practice

The implementation relies on RoboticsAcademy’s HAL and WebGUI interfaces. The main functions are:

- `HAL.getPose3d()` — Returns the robot pose `(x, y, yaw)`.
- `HAL.setV(v)` — Sets linear velocity (m/s).
- `HAL.setW(w)` — Sets angular velocity (rad/s).
- `WebGUI.getMap(path)` — Loads the map image from the specified path.
- `WebGUI.showNumpy(image)` — Displays a numpy image in the GUI (visual map).

---

## 1. Map Loading and Preprocessing

The map is loaded and processed to obtain a clean binary image separating obstacles from free space.

```python
raw_image = WebGUI.getMap(MAP_PATH)
gray_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
_, binary_image = cv2.threshold(gray_image, 250, 255, cv2.THRESH_BINARY)
kernel = np.ones((11, 11), np.uint8)
eroded_image = cv2.erode(binary_image, kernel, iterations=2)
map_image = cv2.resize(eroded_image, (1000, 1000), interpolation=cv2.INTER_NEAREST)
```

**Design choices:**
- High threshold (250) to separate bright floor from obstacles.
- Morphological erosion to remove noise.
- Resize to a uniform `1000x1000` grid for consistent cell processing.

---

## 2. Coordinate Transformations

A fixed transformation matrix is used to convert between *world coordinates (meters)* and *pixel coordinates* of the map.

```python
PIXELS_PER_METER = 102
OFFSET_X = -5.6076
OFFSET_Y = -4.021

TRANSFORM = np.array([
    [-PIXELS_PER_METER, 0, -OFFSET_X * PIXELS_PER_METER],
    [0, PIXELS_PER_METER, -OFFSET_Y * PIXELS_PER_METER],
    [0, 0, 1]
])
INV_TRANSFORM = np.linalg.inv(TRANSFORM)
```

Helper functions provide conversions:
- `get_robot_pixel()` → world → pixels.
- `pixel_to_grid()` and `grid_to_pixel()` → pixels ↔ grid indices.
- `pixel_to_world()` → pixels → world.

These conversions ensure precise movement planning and visualization.

---

## 3. Grid Construction

The binary map is divided into uniform cells of `CELL_SIZE`, each classified as either obstacle or target (free).

```python
for r in range(rows):
    for c in range(cols):
        cell = map_image[r*CELL_SIZE:(r+1)*CELL_SIZE, c*CELL_SIZE:(c+1)*CELL_SIZE]
        if np.any(cell == 0):
            grid_obstacles.add((r, c))
            mark_cell(r, c, COLOR_OBSTACLE)
        else:
            grid_targets.add((r, c))
            mark_cell(r, c, COLOR_FREE)
```

**Data structures:**
- `grid_obstacles` → occupied cells.  
- `grid_targets` → cells to be cleaned.  
- `grid_visited` → cells already visited.  
- `return_points` → branches to revisit if needed.

---

## 4. Search Algorithms (BFS)

Breadth-First Search (BFS) is implemented for two purposes:

1. `bfs(start, goal)` — path between two cells.
2. `bfs_nearest_target(start, targets)` — shortest path to the nearest unvisited cell.

Simplified BFS code:

```python
def bfs(start, goal):
    if start == goal:
        return []
    queue = deque([start])
    parents = {start: None}
    while queue:
        current = queue.popleft()
        if current == goal:
            break
        for neighbor in get_neighbors(current):
            if neighbor in grid_obstacles or neighbor in parents:
                continue
            parents[neighbor] = current
            queue.append(neighbor)
    path = []
    node = goal
    while parents[node] is not None:
        path.append(node)
        node = parents[node]
    path.reverse()
    return path
```

---

## 5. Motion Control to a Grid Cell

The function `move_to_grid_cell(row, col)` performs a basic proportional control:

```python
dx = twx - wx
dy = twy - wy
dist = math.hypot(dx, dy)
angle_goal = math.atan2(dy, dx)
_, _, yaw = get_robot_pose()
angle_diff = (angle_goal - yaw + math.pi) % (2 * math.pi) - math.pi

if abs(angle_diff) > 0.1:
    HAL.setV(0)
    HAL.setW(angle_diff * 0.5)
else:
    HAL.setW(0)
    HAL.setV(0.3)
```

**Behavior:**
- Rotate first until alignment (`angle_diff < 0.1` rad).  
- Move forward with constant speed (0.3 m/s).  
- Stop when distance < 0.1 m.

---

## 6. Exploration Logic and State Machine

The robot operates as a finite state machine with three states:

| State | Description |
|--------|--------------|
| `search` | Finds next target locally, marks return points, detects critical cells. |
| `move` | Follows current BFS path step-by-step. |
| `done` | Stops when all cells are cleaned. |

Main loop pseudocode:

```python
while True:
    cell = pixel_to_grid(px, py)
    mark_visited(cell)
    if robot_state == 'search':
        # compute next_cell
    elif robot_state == 'move':
        if current_path:
            next_cell = current_path[0]
            if move_to_grid_cell(*next_cell):
                current_path.pop(0)
        else:
            robot_state = 'search'
    elif robot_state == 'done':
        print("Cleaning completed.")
        break
```

**Coverage Strategy:**
- Prioritize local exploration (forward/east/west moves).  
- Save unexplored branches in `return_points`.  
- Use BFS to return to them when no local targets remain.

---

## 7. Visualization and Debug in WebGUI

The robot's state is displayed visually using cell color codes:

- `COLOR_OBSTACLE = 0` (black)  
- `COLOR_FREE = 127` (light gray)  
- `COLOR_VISITED = 131` (green)  
- `COLOR_ROBOT = 133` (indigo)  
- `COLOR_RETURN_POINT = 132` (blue)  
- `COLOR_CRITICAL_POINT = 128` (red)

Example update before rendering:

```python
temp_map = visual_map.copy()
mark_cell(*cell, COLOR_ROBOT, img=temp_map)
WebGUI.showNumpy(temp_map)
```

---

## Video Demo
<div align="center">
<img width="860px" src="https://github.com/GuilleAQ/Service-Robots_25-26/blob/main/P1/resources/figures/2.png">
</div>

[video demo at x16 speed](https://urjc-my.sharepoint.com/:v:/g/personal/g_alcocer_2020_alumnos_urjc_es/ESDepUTk_G9BsrUnhz8-7uQBwF1a503OcUpUGkW6uDrYVQ?e=9Iv4vq)

---

