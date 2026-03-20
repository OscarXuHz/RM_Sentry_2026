# Plan 2: Safe Flight Corridor (SFC) for Trajectory Generation

> **Status:** In Progress  
> **Priority:** High  
> **Dependencies:** None  
> **Risk:** Low (static map) / Medium (with dynamic hybrid)  

## Summary

Replace the expensive soft-obstacle-penalty approach in the L-BFGS path smoother (`obstacleTerm()` + 100 ray-casts per control point) with **axis-aligned Safe Flight Corridors (SFC)**. Each path segment gets a pre-computed free-space bounding box. The L-BFGS optimization enforces control points stay inside their corridor box via a quadratic exterior penalty. This eliminates the "most time-consuming part" of the optimization (per the code comment at `path_smooth.cpp` L107: `整个优化过程最最最耗时的地方`) and provides stronger collision guarantees.

**Dynamic obstacles** are handled via a lightweight hybrid approach: SFC for static walls (hard), soft penalty for moving obstacles (kept from current code but simplified).

---

## Current Architecture (Being Replaced)

### Obstacle term in `costFunction()` — `path_smooth.cpp` L120–136

For each of the `N-1` inner control points:

1. **`getObsEdge(xcur)`** (L378–418): Searches ±8 grid cells + 100 radial ray-casts up to 4.0m
2. **`obstacleTerm(idx, xcur)`** (L328–375): Quadratic penalty `w_obs * (R - d)²` for each cached obstacle within threshold
3. Result: cached in `allobs[]` (only sampled on first L-BFGS iteration)

| Parameter | Value |
|-----------|-------|
| `wObstacle` | 1e5 |
| Defense radius `R` | 0.3m |
| Grid search | ±8 cells (±0.4m at 0.05m resolution) |
| Ray-casts | 100 rays × up to 4.0m range |
| Obstacle cache | `allobs[idx]` — `vector<vector<Vector3d>>`, sampled once per `smoothPath()` call |

### Problems

1. **Expensive**: 100 ray-casts per control point per L-BFGS iteration (code comment: "most time-consuming")
2. **Soft constraint**: penalty can be overcome by smoothness cost in narrow passages → collision possible
3. **Fixed radius**: `R=0.3m` doesn't adapt to corridor width
4. **Cache staleness**: obstacles sampled once at initial positions; if control points move far during optimization, cached obstacles may be irrelevant

---

## Design: SFC-Based Obstacle Avoidance

### Algorithm

For each waypoint along the path, generate an axis-aligned bounding box (AABB) representing the maximum free space around that point:

```
For waypoint w_i at grid position (gx, gy):
  1. Initialize box = {gx, gx, gy, gy}  (single cell)
  2. Loop:
     a. Try expand LEFT:  check all cells in column (box.x_min - 1, box.y_min..box.y_max)
        If all free → box.x_min -= 1
     b. Try expand RIGHT: check all cells in column (box.x_max + 1, ...)
        If all free → box.x_max += 1
     c. Try expand DOWN:  check all cells in row (box.x_min..box.x_max, box.y_min - 1)
        If all free → box.y_min -= 1
     d. Try expand UP:    check all cells in row (..., box.y_max + 1)
        If all free → box.y_max += 1
     e. If no face expanded → stop
  3. Shrink by robot_radius to get safe AABB in world coordinates
  4. Cap maximum dimension at 2.0m per side (prevent degenerate large boxes)
```

### Corridor assignment

- Waypoint `i` → Corridor `i`
- Inner control point `i` (between waypoints `i` and `i+1`) → assigned to the **intersection** of corridors `i` and `i+1` (guaranteed non-empty since adjacent waypoints are connected by a free path)
- If intersection is empty (shouldn't happen, but safety): fall back to corridor of the closer waypoint

### Cost function replacement

Instead of `obstacleTerm()`:

```
For control point (x, y) with assigned corridor {x_min, x_max, y_min, y_max}:

penalty = w_sfc * (max(0, x_min - x)² + max(0, x - x_max)²
                 + max(0, y_min - y)² + max(0, y - y_max)²)

gradient_x = w_sfc * (-2*(x_min - x) if x < x_min, +2*(x - x_max) if x > x_max, else 0)
gradient_y = w_sfc * (-2*(y_min - y) if y < y_min, +2*(y - y_max) if y > y_max, else 0)
```

This is a **quadratic exterior penalty** — zero inside the corridor, quadratically increasing outside.

### Dynamic obstacle hybrid

- SFC generated from **static map only** (`data[]` array)
- For dynamic obstacles (`l_data[]`): keep a lightweight soft penalty
  - Search ±4 cells (not ±8) around each control point in `l_data[]` only
  - Same quadratic penalty `w_dyn * (R - d)²` with `R = 0.3m`
  - No ray-casts needed — dynamic obstacles are already close-range
- This hybrid directly answers the original author's concern: "飞行走廊怎么去处理动态障碍物" → don't, keep soft penalty for those

---

## Implementation Steps

### Step 1: Create `sfc_generator.hpp`

**Location:** `trajectory_generation/include/sfc_generator.hpp`

```cpp
struct AABB {
    double x_min, x_max, y_min, y_max;
    bool contains(double x, double y) const;
    AABB intersect(const AABB &other) const;
    bool valid() const;  // x_min < x_max && y_min < y_max
};

class SFCGenerator {
public:
    void setMap(std::shared_ptr<GlobalMap> &map);
    std::vector<AABB> generateCorridors(const std::vector<Eigen::Vector2d> &waypoints);

private:
    std::shared_ptr<GlobalMap> global_map_;
    AABB inflateBox(int gx, int gy) const;
    bool isColumnFree(int x, int y_min, int y_max) const;
    bool isRowFree(int x_min, int x_max, int y) const;
    
    static constexpr double MAX_BOX_HALF = 2.0;    // max 2m per side
    static constexpr double ROBOT_SHRINK = 0.25;    // robot radius inflation
};
```

### Step 2: Implement `sfc_generator.cpp`

**Location:** `trajectory_generation/src/sfc_generator.cpp`

Key implementation details:
- `inflateBox()`: starts at waypoint grid cell, expands greedily in all 4 directions
- Use `global_map_->data[]` only (static map) — NOT `l_data[]`
- After grid expansion, convert grid AABB to world AABB via `gridIndex2coord()`
- Shrink each face inward by `ROBOT_SHRINK` (≈ robot radius)
- For corridor continuity: verify adjacent boxes overlap. If not, extend the smaller box toward the other's centroid until overlap achieved

### Step 3: Integrate SFC into `Smoother`

**File:** `trajectory_generation/include/path_smooth.h`

Changes:
- Add `#include "sfc_generator.hpp"`
- Add member `SFCGenerator sfcGen;`
- Add member `std::vector<AABB> corridors_;`
- Keep `wObstacle` but rename to `wSFC` = 1e5 (same weight)
- Remove: `allobs`, `mid_distance`, `init_obs` members

### Step 4: Modify `smoothPath()` in `path_smooth.cpp`

**File:** `trajectory_generation/src/path_smooth.cpp` L231–296

Before L-BFGS call:
1. Generate corridors: `corridors_ = sfcGen.generateCorridors(path)`
2. Compute corridor assignments for each inner control point (intersection of adjacent corridors)

### Step 5: Replace obstacle cost in `costFunction()`

**File:** `trajectory_generation/src/path_smooth.cpp` L69–141

Replace the obstacle computation block (L116–136) with:
1. Corridor penalty for each inner point (quadratic exterior)
2. Lightweight dynamic obstacle penalty (±4 cell search on `l_data[]` only)
3. Remove: `init_obs` check, `getObsEdge()` call, `allobs` usage

### Step 6: Remove `obstacleTerm()` and `getObsEdge()`

**File:** `trajectory_generation/src/path_smooth.cpp` L328–418

- Delete or comment out `obstacleTerm()`, `getObsEdge()`, `getObsPosition()`
- Keep `isFree()` (used by SFC generator)

### Step 7: Update CMakeLists.txt

**File:** `trajectory_generation/CMakeLists.txt`

- Add `src/sfc_generator.cpp` to the source file list

---

## Verification Plan

| Test | Method | Pass Criteria |
|------|--------|---------------|
| Corridor generation | RViz MarkerArray visualization on `occfinal.png` map | Boxes cover path, don't overlap walls |
| Narrow passage | Path through bridge/gap | Small box generated; path stays inside |
| Timing | Benchmark `costFunction()` × 20 queries | ≥ 3× speedup vs current ray-cast approach |
| Safety | 100 `smoothPath()` runs | No final path point inside static obstacle |
| Dynamic hybrid | Add moving obstacle near path | Path deflects around it via soft penalty |
| Corridor overlap | Log adjacent box overlap for every plan | Overlap ≥ 2 × resolution (0.1m) |
| Regression | Compare smoothed path length/quality vs current | Path length within 5% of current |

---

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Box type | Axis-aligned (AABB) | 2D ground robot; AABB is sufficient, simple to query and intersect |
| Penalty type | Quadratic exterior | L-BFGS handles smooth penalties well; log-barrier undefined when point starts outside |
| SFC source | Static map (`data[]`) only | Dynamic obstacles change every frame; rebuilding SFC each cycle is too expensive |
| Dynamic handling | Hybrid: SFC static + soft penalty dynamic | Addresses original author's concern directly |
| `wSFC` weight | 1e5 (same as current `wObstacle`) | Proven to work; tune later if needed |
| Max box size | 2.0m per side | Prevents degenerate large boxes in open areas; larger boxes don't help optimization |
| Robot shrink | 0.25m | Conservative; matches MPC dynamic clearance threshold |

---

## Files Affected

| File | Action | Estimated Lines |
|------|--------|----------------|
| `trajectory_generation/include/sfc_generator.hpp` | **NEW** | ~60 |
| `trajectory_generation/src/sfc_generator.cpp` | **NEW** | ~150 |
| `trajectory_generation/include/path_smooth.h` | Modify members | ~10 changed |
| `trajectory_generation/src/path_smooth.cpp` | Replace obstacle code | ~80 changed |
| `trajectory_generation/CMakeLists.txt` | Add source file | ~1 line |

**Estimated total new/modified lines:** ~300

---

## Rollback Plan

If SFC produces worse results:
1. The original `obstacleTerm()` / `getObsEdge()` code can be restored from git
2. SFC and soft penalty can coexist: set `wSFC = 0` to disable corridors, `wObstacle > 0` to re-enable soft penalty
3. If specific paths fail with SFC, add per-segment fallback: if corridor is too small (< 2×robot_radius), skip SFC penalty for that point and use original soft penalty instead
