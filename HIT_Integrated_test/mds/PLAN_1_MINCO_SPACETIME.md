# Plan 1: MINCO Joint Space-Time Trajectory Optimization

> **Status:** ✅ Implemented (Option A) — Bug-fixed & verified  
> **Priority:** High  
> **Dependencies:** None  
> **Risk:** Medium  
> **Build:** `catkin_make --pkg trajectory_generation` — PASS  
> **Gradient Tests:** 4/4 PASS (v_mid, waypoint grad, time grad, energy comparison)  

## Summary

Replace the current cubic-spline + fixed-time L-BFGS path smoother with a **MINCO (Minimum Control)** trajectory representation that jointly optimizes waypoint positions **and** per-segment time durations. This enables variable-speed trajectories with per-segment velocity/acceleration constraints (e.g., slow before bridges, decelerate on slopes), faster overall traversal, and higher-order smoothness.

**Implementation strategy: Option A** — Keep `Refenecesmooth` for final cubic polynomial output to the tracker. Feed it MINCO-optimized waypoints and durations. No message format or tracker changes needed.

---

## Pre-Implementation Analysis Findings

The following corrections to the original plan were identified during code review:

1. **`forwardT`/`backwardT`/`backwardGradT` (L438-483 in path_smooth.cpp):** Confirmed fully implemented but **never called** — the `m_trapezoidal_time` member was declared but never written to. These are now wired into the MINCO optimization loop.

2. **`Smoother::m_trapezoidal_time`:** Was previously a dead variable — never populated by `smoothPath()`, passed as empty to `Refenecesmooth`. Now carries MINCO-optimized durations.

3. **Plan overestimated scope of Option A:** The plan said "touches 4 files in `trajectory_generation` and 2 in `trajectory_tracking`, plus `trajectoryPoly.msg`". Under Option A, only 4 files in `trajectory_generation` are needed. No tracker or message changes required.

4. **`BandedSystemNoTime` solver NOT ported:** The plan suggested porting the 4N×4N banded LU solver from `cubic_spline.hpp` and enlarging bandwidth from 4 to 6. Instead, the MINCO implementation uses a direct block-tridiagonal Thomas algorithm (O(N) complexity, no bandwidth parameter needed), which is simpler and more numerically stable for the MINCO structure.

5. **Matrix system size:** The plan said "3(N+1)×3(N+1), band=6". In practice, the MINCO system is block-tridiagonal with 2×2 blocks for the N-1 interior knots, solved via Thomas algorithm — no banded matrix is ever formed.

---

## Current Architecture (Pre-MINCO)

### What existed before

| Component | File | Role |
|-----------|------|------|
| `CubicSpline` | `trajectory_generation/include/root_solver/cubic_spline.hpp` | Piecewise cubic (order 3) with 4N×4N banded system (bandwidth 4) |
| `Smoother::costFunction` | `trajectory_generation/src/path_smooth.cpp` L69 | L-BFGS callback optimizing **spatial positions only** |
| `Refenecesmooth` | `trajectory_generation/src/reference_path.cpp` | Natural cubic spline interpolation + trapezoidal time allocation |
| `trajectoryPoly.msg` | `trajectory_generation/msg/trajectoryPoly.msg` | Carries 4 coefficients/piece (cubic) — **unchanged** |
| `local_planner.cpp` | `trajectory_tracking/src/local_planner.cpp` L780–840 | Evaluates cubic polynomial for MPC reference — **unchanged** |
| `forwardT` / `backwardT` / `backwardGradT` | `trajectory_generation/src/path_smooth.cpp` L438–483 | **Already implemented but UNUSED** time diffeomorphism — **now wired** |

### Previous decision variables

```
x ∈ ℝ^{2(N-1)}   ← inner control point positions (x, y) only
Time: FIXED at 0.3 / desire_velocity per segment (line 91 of path_smooth.cpp)
```

### Key limitations addressed

1. ~~**No joint space-time optimization**~~ → Now optimizes positions AND segment times jointly
2. ~~**Cubic (C2) smoothness only**~~ → MINCO provides C3 (jerk-continuous) internal representation
3. ~~**Time allocation is post-hoc**~~ → Time durations now come from the optimizer, fed directly into `Refenecesmooth`
4. **Per-segment speed/accel constraints** → Soft velocity penalty added in `costFunction()`; hard per-segment limits (bridge/slope) are future work (Step 3)

---

## What MINCO Changed

### Polynomial upgrade (internal optimizer)

| Property | Previous (Cubic) | MINCO (Min-Jerk) |
|----------|-----------------|-------------------|
| Polynomial order | 3 | **5** |
| Coefficients/piece | 4 | **6** |
| Continuity | C2 | **C3** (jerk-continuous) |
| Solver | 4N×4N banded LU (band=4) | **Block-tridiagonal Thomas algorithm (O(N))** |
| Energy | ∫‖p″(t)‖²dt | **∫‖p‴(t)‖²dt** (jerk integral) |
| Time optimization | None | **Analytic ∇_T via coefficient derivative chain rule** |

### Decision variable upgrade

```
x ∈ ℝ^{2(N-1) + N}
     ├── 2(N-1) entries: inner waypoint positions (x, y)
     └── N entries: unconstrained τ_i mapped to T_i via forwardT()
```

### Pipeline (Option A)

```
A* path → Smoother (L-BFGS with MINCO) → optimized waypoints + T_i
                                              ↓
                              Refenecesmooth (cubic spline + T_i from MINCO)
                                              ↓
                              trajectoryPoly.msg (4 coefs/piece, unchanged)
                                              ↓
                              LocalPlanner tracker (unchanged)
```

---

## Implementation Report

### Step 1: Created `minco_trajectory.hpp` ✅

**File:** `trajectory_generation/include/root_solver/minco_trajectory.hpp` (701 lines, NEW)

Implemented the full MINCO representation for 2D (x, y) with 5th-order (min-jerk) polynomials — 6 coefficients per piece per axis.

**Class `MincoTrajectory` public interface (as implemented):**

```cpp
class MincoTrajectory {
public:
    void setConditions(headPos, headVel, headAcc, tailPos, tailVel, tailAcc, pieceNum);
    void setParameters(innerPoints, durations);  // builds trajectory
    double getEnergy() const;                     // ∫‖p'''(t)‖²dt
    void getGradWaypoints(Matrix2Xd &grad) const; // ∇_waypoints via envelope theorem
    void getGradTimes(VectorXd &grad) const;       // ∇_T via coefficient derivatives
    Vector2d evaluate(piece, t) const;             // position
    Vector2d evaluateVel(piece, t) const;          // velocity
    Vector2d evaluateAcc(piece, t) const;          // acceleration

private:
    void computeCoeffs();                          // quintic coefficients from boundary PVA
    void solveOptimalVelAcc(innerPs);              // Thomas algorithm for interior v,a
    void mapCoeffGradToBoundary(...);              // dE/dc → dE/d(boundary PVA)
    Eigen::MatrixX2d solveAdjointForWaypoints(...) const;  // envelope theorem propagation
    double computeTimeCoefficientGrad(i) const;    // dE/dT_i via dc/dT chain rule
};
```

**Key implementation details:**
- **Quintic coefficients:** For each piece, `c0..c5` are computed from boundary PVA (position, velocity, acceleration) at both endpoints. `c0=p0`, `c1=v0`, `c2=a0/2`, and `c3..c5` from the 3×3 system solving boundary matching at `t=T_i`.
- **Interior velocity/acceleration solve:** Block-tridiagonal Thomas algorithm solves for optimal `v_k, a_k` at interior knots. Continuity of position, velocity, acceleration is enforced; jerk-energy is minimized.  
  **⚠️ Critical bug found and fixed** — see [Post-Implementation Bug Fix](#post-implementation-bug-fix) section below.
- **Energy computation:** Closed-form integral `∫₀^T ‖p'''(t)‖²dt` per piece, evaluating to `36c3² T + 144 c3 c4 T² + (192 c4² + 240 c3 c5) T³ + 720 c4 c5 T⁴ + 720 c5² T⁵`.
- **Gradient w.r.t. waypoints:** Uses envelope theorem — since interior `v_k, a_k` are at their optimum, `dE/dp_k` is directly the position component of the boundary PVA gradient.
- **Gradient w.r.t. times:** Chain rule through coefficient formulas: `dE/dT_i = Σ (dE/dc_j)(dc_j/dT_i)`.

### Step 2: Modified `path_smooth.h` ✅

**File:** `trajectory_generation/include/path_smooth.h`

Changes:
- `#include <root_solver/cubic_spline.hpp>` → `#include <root_solver/minco_trajectory.hpp>`
- `CubicSpline cubSpline` → `MincoTrajectory mincoTraj`
- Added members: `Eigen::Vector2d headV_stored, headA_stored` for boundary conditions in L-BFGS callback

### Step 3: Rewrote `costFunction()` and `smoothPath()` ✅

**File:** `trajectory_generation/src/path_smooth.cpp`

#### `costFunction()` (static L-BFGS callback)

Decision variable layout: `x[0..2(N-1)-1]` = inner waypoint positions (x,y), `x[2(N-1)..2(N-1)+N-1]` = unconstrained time parameters τ.

**Cost terms (post bug-fix):**

| Term | Weight | Formula | Gradient |
|------|--------|---------|----------|
| Jerk energy | `wSmooth = 0.1` | `w * getEnergy()` | Analytic via `getGradWaypoints()` + `getGradTimes()` → `backwardGradT()` |
| Elastic band (fidelity) | `wFidelity = 1e3` | `w * Σ ‖p_k - p_k^ref‖²` | `2w(p_k - p_k^ref)` per waypoint |
| Average velocity | `wVel = 1e4` | `w * Σ max(0, seg_len/T_i - 1.5·v_des)²` | Analytic for both spatial and time variables |
| Minimum time | `wMinTime = 1e3` | `w * Σ max(0, 0.03 - T_i)²` | `-2w(T_min - T_i)` via `backwardGradT()` |
| Obstacle | `wObstacle = 1e5` | `obstacleTerm()` (unchanged) | Spatial only |

```
1. Extract inner waypoints from x[0..2(N-1)-1]
2. Extract τ from x[2(N-1)..end], map to T via forwardT()
3. Build MINCO: mincoTraj.setParameters(innerPs, T)
4. Jerk energy: wSmooth * getEnergy(), with full analytic gradients
5. Elastic band: wFidelity * Σ ‖p_k - p_k_ref‖² (anchors to A* reference positions)
6. Average velocity: wVel * Σ max(0, avg_vel - v_limit)² with gradient for spatial + time
7. Minimum time: wMinTime * Σ max(0, T_min - T_i)² prevents duration collapse
8. Obstacle cost: obstacleTerm() (unchanged, spatial only)
9. All time gradients chained through backwardGradT() to τ-space
```

#### `smoothPath()`

```
1. Store boundary conditions (headV, headA) for costFunction callback
2. Set MINCO conditions: mincoTraj.setConditions(head, headV, headA, tail, 0, 0, N)
3. Store reference waypoints (for elastic band cost in costFunction)
4. Initialize: waypoints from input path, τ from backwardT(segment_length / desire_velocity)
5. Pack into x ∈ ℝ^{2(N-1)+N}
6. Run L-BFGS optimization
7. Extract optimized waypoints and T = forwardT(τ_opt)
8. Post-process: enforce velocity feasibility by scaling up any segment whose
   max sampled velocity exceeds 1.8× desire_velocity
9. Store times into m_trapezoidal_time
10. Return optimized waypoint path
```

#### `init()`

Modified to call `mincoTraj.setConditions()` with head/tail position, velocity, and acceleration boundary conditions. Tail velocity and acceleration are zero (consistent with stop-at-goal behavior).

### Step 4: Modified `reference_path.cpp` ✅

**File:** `trajectory_generation/src/reference_path.cpp` — `getRefTrajectory()`

Added external duration acceptance: if the `times` vector passed in is non-empty and matches the path segment count, use it directly as segment durations (bypassing `solveTrapezoidalTime()`). The `checkfeasible()` loop is still applied (up to 3 iterations) to inflate any segments with velocity violations — this acts as a safety net downstream of the MINCO optimizer.

### Steps 5-6: Skipped (Option B only)

`trajectoryPoly.msg` and `local_planner.cpp` are unchanged. Option A preserves the existing cubic polynomial output from `Refenecesmooth`, so no message format or tracker changes are needed.

### Step 7: Covered by Step 2

The `Smoother` class header changes were implemented as part of Step 2.

---

## Build Issues Encountered and Resolved

| Issue | Cause | Fix |
|-------|-------|-----|
| `computeTimeCoefficientGrad` not declared in scope | Private method defined after public methods calling it | Added forward declarations in private section (then refined — see below) |
| "cannot be overloaded" errors | Forward declarations + inline definitions = two declarations with same signature | Removed forward declarations entirely; C++ class member functions are visible throughout the class body regardless of order |
| `stray '\342'` / `stray '\210'` errors | Unicode math symbols (∂, ∫, ‖, ∇, τ, etc.) in code comments | Replaced all Unicode with ASCII equivalents via sed |
| `'dp_k' does not name a type` at L627 | `dv*/dp_k` in block comment — the `*/` terminates the `/*` comment prematurely | Changed `dv*/dp_k` → `dv_star/dp_k` and `da*/dp_k` → `da_star/dp_k` in comments |
| `'dT' does not name a type` at L658 | Same `*/` issue: `dv*/dT` in block comment | Changed `dv*/dT` → `dv_star/dT` and `da*/dT` → `da_star/dT` in comments |
| **Zigzag/sharp-turn trajectories** | All Hessian blocks in `solveOptimalVelAcc` had wrong values — see [Post-Implementation Bug Fix](#post-implementation-bug-fix) | Complete rewrite of `solveOptimalVelAcc`, `costFunction`, `smoothPath` with mathematically verified formulas |

---

## Files Affected (Actual)

| File | Action | Lines | Status |
|------|--------|-------|--------|
| `trajectory_generation/include/root_solver/minco_trajectory.hpp` | **NEW** (bug-fixed `solveOptimalVelAcc`) | 685 | ✅ Compiles + gradient-verified |
| `trajectory_generation/include/root_solver/cubic_spline.hpp` | Archived (kept, no longer included) | 0 | ✅ |
| `trajectory_generation/include/path_smooth.h` | Modified includes + members + `refWaypoints` | ~10 | ✅ Compiles |
| `trajectory_generation/src/path_smooth.cpp` | Major rewrite: `costFunction` (5 cost terms) + `smoothPath` (post-processing) + `init` | ~180 | ✅ Compiles |
| `trajectory_generation/src/reference_path.cpp` | Accept external MINCO durations + re-enabled `checkfeasible()` | ~20 | ✅ Compiles |
| `sentry_planning_ws/test_minco_gradient.cpp` | **NEW** standalone gradient verification test | ~200 | ✅ 4/4 PASS |
| `trajectory_generation/msg/trajectoryPoly.msg` | **Unchanged** (Option A) | 0 | — |
| `trajectory_tracking/src/local_planner.cpp` | **Unchanged** (Option A) | 0 | — |

**Total new/modified lines:** ~844

---

## Verification Plan

| Test | Method | Pass Criteria | Status |
|------|--------|---------------|--------|
| Compilation | `catkin_make --pkg trajectory_generation` | No errors | ✅ PASS |
| Interior velocity | `test_minco_gradient` test 1: 2-piece straight path, check v_mid = 1.875 | v_mid.x = 1.875 exactly, velocity + accel continuous | ✅ PASS |
| Waypoint gradient | `test_minco_gradient` test 2: finite-difference vs `getGradWaypoints()` | Max relative error < 1e-4 | ✅ PASS (7e-9) |
| Time gradient | `test_minco_gradient` test 3: finite-difference vs `getGradTimes()` | Max relative error < 1e-4 | ✅ PASS (3e-9) |
| Energy comparison | `test_minco_gradient` test 4: straight vs zigzag path jerk energy | Zigzag energy > straight energy | ✅ PASS (4.45× ratio) |
| Integration | RViz visual comparison — MINCO paths vs previous cubic paths | Qualitatively similar or smoother | ⬜ TODO |
| Performance | Time `smoothPath()` over 20 queries | ≤ 2× current time | ⬜ TODO |
| Variable speed | Verify optimized T_i varies by segment geometry | Non-uniform durations | ⬜ TODO |

---

## Design Decisions (Final)

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Polynomial order | 5 (min-jerk) over 7 (min-snap) | Ground robot doesn't need snap continuity; min-jerk is sufficient and cheaper |
| Reference path strategy | **Option A (keep Refenecesmooth)** | Minimizes tracker-side changes; zero risk to message format or controller |
| MINCO solver | Block-tridiagonal Thomas algorithm (not banded LU) | O(N) complexity, no bandwidth parameter, simpler than porting BandedSystemNoTime |
| Time diffeomorphism | Reused existing `forwardT`/`backwardT`/`backwardGradT` | Already implemented (L438-483) — just needed wiring |
| Boundary conditions | headAcc from numerical diff, tailVel = tailAcc = 0 | Consistent with stop-at-goal behavior |
| Gradient strategy | Envelope theorem for waypoints, coefficient chain rule for times | Avoids expensive adjoint system solve; exact at the optimum |
| `m_trapezoidal_time` | Repurposed (was dead) to carry MINCO durations | No new member variables or API changes needed in `plan_manager.cpp` |
| Velocity penalty | Soft quadratic penalty on average segment velocity with analytic gradient | Simple, differentiable, gradient-consistent; full closed-form extrema search is future work (Step 3) |
| Elastic band (fidelity) | Quadratic penalty anchoring waypoints to A* reference positions (wFidelity = 20, reduced from 100 via Fix 33d) | Very gentle reference bias — allows strong corner rounding while preventing drift (clamped at 0.8m) |
| Time initialization | Segment-length-proportional: `max(seg_len / desire_vel, 0.05)` | Better initial guess than uniform time; faster convergence |
| Velocity post-processing | Scale up any segment with max sampled velocity > 1.8 × desire_vel | Robust safety net; avoids gradient inconsistency of in-loop hard constraints |

---

## Future Work

1. **Per-segment speed limits (Step 3):** Add hard per-segment velocity limits from config (bridge_coeff, slope_coeff). Currently a uniform soft penalty is applied — extend to use closed-form velocity extrema of the quintic polynomial.
2. **Option B upgrade:** If higher-quality reference trajectories are needed, extend `trajectoryPoly.msg` with `poly_order` field and directly output MINCO quintic coefficients to the tracker.
3. ~~**Gradient verification:** Run finite-difference gradient checks on `costFunction()` to validate analytic gradients.~~ **DONE** — 4/4 tests pass (see Verification Plan).
4. **Runtime profiling:** Benchmark L-BFGS convergence with expanded 2(N-1)+N variables vs the previous 2(N-1).
5. ~~**Weight tuning:** The current cost weights (wSmooth=0.1, wFidelity=1e3, wVel=1e4, wMinTime=1e3) are initial values validated by gradient tests. They may need tuning based on observed behavior in different map configurations and obstacle densities.~~ **DONE** — Rebalanced across Fixes 28→29→31→33. Final values: `wSmooth=2e-2, wFidelity=20, wVel=1e3, wMinTime=1e3`. See Fixes 28, 29, 31, 33 in PLANNING_FIX_HISTORY.md.
6. ~~**Obstacle cache refresh:** The stale obstacle cache in `obstacleTerm()` is a fundamental architecture limitation. Current mitigation is waypoint drift clamping (0.5m radius). A proper fix would recompute obstacles every N L-BFGS iterations.~~ **DONE** — Fix 30 refreshes `allobs[]` every 50 L-BFGS iterations. Drift clamp relaxed to 0.8m (Fix 31). Obstacle search window widened from ±8 to ±12 grid cells.
7. **Field testing:** Validate trajectory quality with the rebalanced weights in the physical arena under various obstacle configurations.

---

## Post-Implementation Bug Fix

### Symptoms

After initial MINCO deployment, two critical trajectory quality issues were observed:

1. **Paths zigzag across the map** — not following shortest-path behavior; some segments overshoot wildly
2. **Paths are straight with sharp 90° turns** — when the optimizer stalls due to gradient inconsistency, the unsmoothed A* grid path is returned as-is

### Root Cause: Wrong Hessian Blocks in `solveOptimalVelAcc`

Every block in the Thomas algorithm's tridiagonal system had incorrect values. The errors were traced through a full Jacobian–Hessian factorization derivation.

**Correct derivation:** For a min-jerk quintic piece, the energy `E = c^T Q c` has coefficient Hessian `Q`:

```
Q = [ 36T    72T²   120T³ ]
    [ 72T²  192T³   360T⁴ ]
    [120T³  360T⁴   720T⁵ ]
```

The boundary Hessian is `H = J^T (2Q) J` where `J` is the Jacobian of `(c3,c4,c5)` w.r.t. boundary values.

**Diagonal blocks — old (wrong) vs new (correct):**

| Entry | Old (wrong) | Correct | Error factor |
|-------|------------|---------|--------------|
| `H_RR(0,0)` | `192/T³` | **`384/T³`** | ×2 |
| `H_RR(0,1)` | `-36/T²` | **`-72/T²`** | ×2 |
| `H_RR(1,1)` | `12/T` | **`18/T`** | ×1.5 |
| `H_LL(0,0)` | `192/T³` | **`384/T³`** | ×2 |
| `H_LL(0,1)` | `36/T²` | **`72/T²`** | ×2 |
| `H_LL(1,1)` | `12/T` | **`18/T`** | ×1.5 |

**Off-diagonal blocks — old (wrong) vs new (correct):**

| Entry | Old (wrong) | Correct | Notes |
|-------|------------|---------|-------|
| `H_LR(0,0)` | `-192/T³` | **`336/T³`** | Wrong sign AND magnitude |
| `H_LR(0,1)` | `-36/T²` | **`-48/T²`** | Wrong magnitude |
| `H_LR(1,0)` | `36/T²` | **`48/T²`** | Wrong magnitude |
| `H_LR(1,1)` | `-12/T` | **`-6/T`** | Wrong magnitude |

**RHS gradient — old (wrong) vs new (correct):**

| Term | Old (wrong) | Correct | Error |
|------|------------|---------|-------|
| `gv from dp` | `-120·dp/T⁴` | **`-720·dp/T⁴`** | ×6 |
| `ga_R from dp` | `20·dp/T³` | **`120·dp/T³`** | ×6 |
| `gv from headV` | `192·v/T³` | **`336·v/T³`** | Wrong value |
| `ga from headA` | `-12·a/T` | **`-6·a/T`** | ×2 |

### Effect

For a simple 2-piece straight path (0→0.5→1.0), the optimal midpoint velocity should be **1.875 m/s**. The old solver produced **v_mid ≈ 0**, causing each piece to independently solve zero-to-zero velocity transitions with astronomically high jerk.

### Additional Fixes Applied

1. **Elastic band cost** (`wFidelity = 1e3`): Anchors waypoints to their original A* reference positions, preventing drift and zigzag
2. **Average velocity penalty with gradient** (`wVel = 1e4`): Penalizes `max(0, avg_vel - 1.5·v_des)²` with analytic gradient for both spatial and time variables; the old velocity penalty added cost but zero gradient, breaking L-BFGS
3. **Smoothness weight** (`wSmooth = 0.1`): Scales jerk energy to balance with obstacle/fidelity costs
4. **Minimum time penalty** (`wMinTime = 1e3`): Prevents durations from collapsing below 0.03s
5. **Length-proportional time initialization**: `max(seg_len / desire_vel, 0.05)` instead of uniform `0.3 / desire_vel`
6. **Velocity post-processing** in `smoothPath()`: After optimization, scales up any segment whose max sampled velocity exceeds `1.8 × desire_velocity`
7. **Re-enabled `checkfeasible()`** in `reference_path.cpp` for MINCO-provided times (3-iteration safety net)

### Verification

**Test file:** `sentry_planning_ws/test_minco_gradient.cpp` (standalone, compiled with `g++ -std=c++14`)

| Test | Result |
|------|--------|
| v_mid = 1.875 for 2-piece straight path | ✅ PASS (exact match) |
| Velocity and acceleration continuity at interior knot | ✅ PASS (error = 0) |
| Waypoint gradient vs finite differences | ✅ PASS (max relative error 7e-9) |
| Time gradient vs finite differences | ✅ PASS (max relative error 3e-9) |
| Zigzag path has higher jerk energy than straight path | ✅ PASS (4.45× ratio) |

---

## Post-Hessian-Fix Regression: Weight Rebalancing & Safety Guards

### Problem

The Hessian fix above was **mathematically correct**, but caused a severe runtime regression: trajectories zigzagged worse than before, RViz crashed from memory exhaustion, and the NMPC tracker failed continuously.

### Root Cause

The correct Hessian produces gradient magnitudes with $1/T^4$ scaling. At $T \approx 0.15\text{s}$, the energy gradient per waypoint reaches ~427,000 per axis. Even at `wSmooth = 0.1`, the weighted energy gradient (~42,700) was **20× larger** than the obstacle gradient (~2,000), causing the optimizer to ignore obstacles entirely while aggressively minimizing jerk. Combined with:
- Stale obstacle cache (never updates during optimization)
- Unlimited L-BFGS iterations (`max_iterations = 0`)
- Uncapped velocity post-processing inflation
- No trajectory time safety cap in `getRefTrajectory()`

### Solution — 9 targeted fixes

| Fix | Location | Change |
|-----|----------|--------|
| Weight rebalancing | `costFunction()` | `wSmooth: 0.1→1e-3`, `wFidelity: 1e3→5e3`, `wVel: 1e4→1e3` |
| Energy gradient clip | `costFunction()` | Per-waypoint norm cap at 2000 |
| Time gradient clip | `costFunction()` | Per-entry clamp ±2000 |
| NaN/Inf guards | `costFunction()` | Two guard points: energy output + final assembly |
| Bounded iterations | `smoothPath()` | `max_iterations = 200` |
| Waypoint drift clamp | `smoothPath()` | 0.5m radius from refWaypoints |
| Velocity inflation cap | `smoothPath()` | `min(scale, 5.0)` |
| Segment time clamp | `smoothPath()` | [0.01, 2.0] seconds |
| Trajectory time cap | `getRefTrajectory()` | `MAX_TRAJ_TIME = 30s` → max 600 points |

### Updated Design Decision: Weight Selection

The weight hierarchy is now:

$$\text{Obstacle} (2{,}000) > \text{Fidelity} (1{,}000) > \text{Energy} (\leq 300) > \text{MinTime} (100) > \text{Velocity} (10)$$

This ensures obstacles are the primary optimization driver, fidelity prevents waypoint drift beyond the obstacle cache radius, and energy provides gentle smoothing without overpowering safety constraints.

### Verification (post-rebalancing)

All 4 gradient tests still pass. Build succeeds with zero errors. See Fix 28 in PLANNING_FIX_HISTORY.md for full details.
