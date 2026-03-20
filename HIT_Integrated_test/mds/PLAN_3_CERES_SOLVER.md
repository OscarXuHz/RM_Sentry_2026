# Plan 3: Replace L-BFGS with Ceres Solver

> **Status:** Planned  
> **Priority:** Low  
> **Dependencies:** Best combined with Plan 2 (SFC corridors)  
> **Risk:** Low  

## Summary

Replace the custom 502-line L-BFGS implementation (`lbfgs.hpp`) in the path smoother with **Google Ceres Solver**. The primary motivation from the HIT document is access to more robust optimization algorithms and automatic differentiation.

**However, this is the lowest-priority upgrade.** The current L-BFGS is well-suited for the unconstrained smooth problem, and Ceres adds ~200MB of dependencies. The main scenario where Ceres genuinely helps is **combined with Plan 2 (SFC)**: Ceres provides native box constraints (`SetParameterLowerBound`/`SetParameterUpperBound`) for corridor enforcement without penalty functions.

---

## Current Architecture

### What exists today

| Component | File | Description |
|-----------|------|-------------|
| `lbfgs.hpp` | `trajectory_generation/include/root_solver/lbfgs.hpp` | 502-line custom L-BFGS with Wolfe line search |
| `costFunction()` | `trajectory_generation/src/path_smooth.cpp` L69 | Static callback: returns cost + gradient as single scalar + VectorXd |
| L-BFGS params | `path_smooth.cpp` L248–254 | `mem_size=64, g_epsilon=2e-5, delta=2e-5, max_linesearch=32` |

### Problem size

```
Variables: 2*(N-1)  where N ≈ 10–30 segments
         → 18–58 optimization variables
```

This is a **tiny** problem by optimization standards. Ceres's framework overhead (trust-region machinery, automatic differentiation infrastructure, memory management) is overkill for this scale.

### Why the original author suggested Ceres

The comment "ceres里面的算法他不香么" likely reflects frustration with L-BFGS convergence in narrow passages or near obstacles where the cost landscape has sharp gradients. Ceres's trust-region solver handles ill-conditioned Hessians better, and its `SetParameterBounds` would enforce corridor constraints natively.

---

## Honest Assessment

### When Ceres helps

| Scenario | L-BFGS | Ceres | Winner |
|----------|--------|-------|--------|
| Unconstrained smooth problem | Native, fast | Same (LINE_SEARCH + LBFGS) | L-BFGS (no overhead) |
| Box-constrained (SFC corridors) | Penalty function | `SetParameterBounds` | **Ceres** |
| Ill-conditioned near obstacles | May oscillate | Trust-region dampens | **Ceres** |
| Very small problem (N<10) | Fast | Overhead dominates | L-BFGS |
| Large problem (N>50) | Fast | Fast | Tie |
| Automatic differentiation needed | N/A | Useful | **Ceres** |
| Dependency footprint | Zero | ~200MB + glog + gflags + SuiteSparse | L-BFGS |

### When Ceres does NOT help

- The smoothness cost gradient is already analytical (from `CubicSpline::getGradSmooth()`)
- The obstacle gradient is already analytical (from `obstacleTerm()`)
- The problem is small enough that solver setup time dominates solve time
- Adding Ceres to a competition robot's embedded system is heavy

---

## Implementation Steps

### Step 1: Install Ceres dependencies

```bash
sudo apt install libceres-dev libglog-dev libgflags-dev libsuitesparse-dev
# Verify:
pkg-config --modversion ceres  # should return ≥ 2.0
```

**Warning:** This pulls in ~200MB of dependencies. For embedded/competition robots, evaluate if this is acceptable.

### Step 2: Update build system

**`trajectory_generation/CMakeLists.txt`:**
```cmake
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(trajectory_generation ${CERES_LIBRARIES})
```

**`trajectory_generation/package.xml`:**
```xml
<depend>libceres-dev</depend>
```

### Step 3: Create Ceres cost functors

**New file:** `trajectory_generation/include/root_solver/ceres_costs.hpp`

```cpp
// Wraps the CubicSpline (or MincoTrajectory) energy computation
struct SmoothnessCostFunctor {
    Smoother* instance;
    explicit SmoothnessCostFunctor(Smoother* s) : instance(s) {}
    bool operator()(const double* const x, double* residual) const;
    // Uses instance->cubSpline internally
};

// Per control-point obstacle/corridor penalty (if NOT using SetParameterBounds)
struct ObstacleCostFunctor {
    int point_idx;
    Smoother* instance;
    bool operator()(const double* const x, double* residual) const;
};
```

Use `ceres::NumericDiffCostFunction` initially — the banded solve inside CubicSpline is non-trivial to express through Ceres AutoDiff. Migrate to analytic Jacobian later if performance is an issue.

### Step 4: Rewrite `smoothPath()`

Replace L-BFGS call with Ceres `Problem`:

```cpp
void Smoother::smoothPath() {
    ceres::Problem problem;
    std::vector<double> x(2 * (pieceN - 1));
    // Fill initial values from path[]...

    // Smoothness cost
    auto* smoothness = new ceres::NumericDiffCostFunction<SmoothnessCostFunctor, 
                           ceres::CENTRAL, 1, DYNAMIC>(
        new SmoothnessCostFunctor(this), ceres::TAKE_OWNERSHIP, 1, x.size());
    problem.AddResidualBlock(smoothness, nullptr, x.data());

    // Option A: Obstacle soft penalty (without SFC)
    for (int i = 0; i < pieceN - 1; i++) {
        problem.AddResidualBlock(obstacle_cost_i, nullptr, x.data());
    }

    // Option B: Box constraints from SFC (with Plan 2)
    for (int i = 0; i < pieceN - 1; i++) {
        problem.SetParameterLowerBound(x.data(), i, corridors_[i].x_min);
        problem.SetParameterUpperBound(x.data(), i, corridors_[i].x_max);
        problem.SetParameterLowerBound(x.data(), i + pieceN - 1, corridors_[i].y_min);
        problem.SetParameterUpperBound(x.data(), i + pieceN - 1, corridors_[i].y_max);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_type = ceres::LINE_SEARCH;        // drop-in for L-BFGS
    options.line_search_direction_type = ceres::LBFGS;
    options.max_num_iterations = 100;
    options.function_tolerance = 2e-5;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}
```

### Step 5: Alternative — Trust Region for constrained formulation

If SFC corridors (Plan 2) are implemented:

```cpp
options.minimizer_type = ceres::TRUST_REGION;    // handles bounds natively
options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
```

This is where Ceres **actually adds value**: trust-region with native bound constraints replaces penalty-based corridor enforcement entirely.

### Step 6: Archive `lbfgs.hpp`

- Keep file for reference
- Remove `#include <root_solver/lbfgs.hpp>` from `path_smooth.h` (L11)
- Remove `lbfgs::lbfgs_parameter lbfgs_params` member (L77)

---

## Verification Plan

| Test | Method | Pass Criteria |
|------|--------|---------------|
| Build | `catkin_make` with Ceres linked | No symbol conflicts with OCS2 |
| Regression | Same 20 test paths, compare cost values | Final cost within 1% of L-BFGS |
| Timing | Benchmark `smoothPath()` × 20 | Within 2× of L-BFGS time (overhead acceptable) |
| Bounds (with SFC) | Log control point positions | All within corridor bounds |
| Edge cases | Narrow passages that caused L-BFGS issues | Ceres converges (trust-region dampens oscillation) |

---

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Solver mode | LINE_SEARCH + LBFGS for drop-in; TRUST_REGION if combined with SFC | Matches current behavior; upgrades with constraints |
| Differentiation | NumericDiffCostFunction initially | Banded solve hard to express in AutoDiff; migrate to analytic later |
| Standalone value | **Not recommended** | Only implement if combined with Plan 2 (SFC box constraints) |

---

## Recommendation

**Only implement this plan if combined with Plan 2 (SFC).** As a standalone change:
- Ceres LINE_SEARCH + LBFGS is functionally equivalent to the current custom L-BFGS
- The dependency cost (~200MB) is not justified for marginal robustness improvement
- The problem is too small (18–58 variables) to benefit from Ceres's advanced features

Combined with SFC, Ceres provides clean bound-constrained optimization that eliminates penalty function tuning entirely.

---

## Files Affected

| File | Action | Lines |
|------|--------|-------|
| `trajectory_generation/include/root_solver/ceres_costs.hpp` | **NEW** | ~80 |
| `trajectory_generation/include/path_smooth.h` | Remove L-BFGS includes/members | ~5 |
| `trajectory_generation/src/path_smooth.cpp` | Replace `smoothPath()` | ~40 |
| `trajectory_generation/CMakeLists.txt` | Add Ceres dependency | ~3 |
| `trajectory_generation/package.xml` | Add Ceres depend | ~1 |

**Estimated total new/modified lines:** ~130
