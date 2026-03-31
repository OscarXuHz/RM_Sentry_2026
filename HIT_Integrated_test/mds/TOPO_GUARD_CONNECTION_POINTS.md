# Topological Guard Points & Connection Points — Technical Report

## Overview

The topological path search (`TopoSearcher`) builds a sparse graph (PRM-like) over the
competition map, then runs Dijkstra to find the shortest obstacle-free path from start to
end.  The graph consists of two node types:

| Type | `NODE_TYPE` | Role |
|------|-------------|------|
| **Guard** | `Guard = 1` | Landmark nodes in free space — represent distinct regions of the map |
| **Connector** | `Connector = 2` | Bridge nodes linking exactly two guard nodes that are mutually visible through the connector |

Source: [`TopoSearch.h`](../../HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/include/TopoSearch.h) lines 22–53,
[`TopoSearch.cpp`](../../HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/src/TopoSearch.cpp).

---

## Why Two Node Types?

The dual-node architecture (Guard + Connector) is an application of the **Visibility-based
Probabilistic Roadmap (Visibility PRM)** paradigm, chosen for three reasons:

1. **Graph sparsity without losing coverage.**  Guards are placed only when a sample
   discovers a new region that no existing guard can see.  This prevents the O(n²)
   edge explosion of a naive PRM where every close pair of samples is connected.
   In our competition map (~12×12 m with corridors and obstacles), a naive PRM with
   ~700 samples generates thousands of edges, making Dijkstra slow and path quality
   inconsistent.  The guard/connector scheme keeps the graph at O(100) nodes and
   O(100) edges while covering every navigable region.

2. **Guaranteed bridging.**  A connector is created only when it can see exactly two
   otherwise-unconnected guards.  This ensures every pair of mutually-invisible
   regions gets connected through at most one intermediate node, without redundant
   edges.  The `needConnection()` check prevents duplicate bridges (§2.2).

3. **Height-aware directionality.**  The competition map contains ramps and
   multi-level areas.  By separating guards (region landmarks) from connectors
   (bridges), the `checkHeightFeasible()` logic (§2.3) can encode directional
   traversability (uphill-only, downhill-only) on connector edges without
   complicating the guard placement logic.

In practice, guards represent "where am I?" and connectors represent "how do I
get between regions?".  Dijkstra then naturally finds the shortest region-hopping
path, and the downstream `smoothTopoPath()` (§6) prunes unnecessary guard
waypoints via line-of-sight shortcuts.

### Density Trade-off (Fix 49→50)

Guard density is controlled by `MIN_GUARD_SPACING` (minimum 2D distance between
any pair of guards).  Too small → graph explodes with redundant guards and slow
Dijkstra.  Too large → sparse graph with too few intermediate waypoints, causing
the downstream polynomial trajectory to overshoot and create loops.

| Parameter | Fix 49 (v1) | Fix 50 (current) | Effect |
|-----------|------------|------------------|--------|
| `MIN_GUARD_SPACING` | 0.5 m | **0.35 m** | +40–60% more guards |
| `max_sample_num` | 500 | **700** | +40% more samples |

The 0.35 m spacing ≈ robot radius provides a natural lower bound: two guards
closer than the robot's physical size add no route-planning value.

---

## 1. Guard Point Generation

Guard points are created during the sampling phase of `createGraph()` (global) or
`createLocalGraph()` (local).  A sample becomes a guard when it **cannot see any existing
guard**, or when additional criteria suggest a new region needs representation.

### 1.1 Seeding from `topo_keypoint`

Before random sampling, the algorithm iterates over `global_map->topo_keypoint` — a
pre-computed set of map keypoints (corridor intersections, chokepoints, etc.):

```cpp
// createGraph(), line ~243
for(int i = 0; i < global_map->topo_keypoint.size(); i++) {
    Eigen::Vector3d pt = global_map->topo_keypoint[i];
    std::vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt, min_distance, exist_unvisiual);

    bool near_terminal = (dist_to_start < 5.0 || dist_to_end < 5.0);

    if (visib_guards.size() < 1) {
        // No guard visible → must be a new region
        m_graph.push_back(new GraphNode(pt, Guard, ++node_id));
    } else if (min_distance > 2.0 || near_terminal) {
        // Nearest guard > 2m away OR near start/end → add for coverage
        m_graph.push_back(new GraphNode(pt, Guard, ++node_id));
    }
}
```

**Decision logic**: A keypoint becomes a guard if (a) no existing guard is visible (brand new
region), or (b) the nearest guard is farther than 2.0 m, or (c) it's within 5.0 m of the
start/end terminals (ensures terminal connectivity).

### 1.2 Random Sampling

After keypoint seeding, up to `max_sample_num = 700` random points are sampled (Fix 50b; was 1000→500→700).
Samples come from `getSample()` which uses `topo_sample_map` (pre-built free-space
point set from the BEV map) with a 10% random fallback in a 12×12 m box around the robot:

```cpp
// getSample(), line ~682
if(m_rand_pos(m_eng) < 0.1) {
    // 10% chance: uniform random in 12×12m around robot
    pt.x() = odom_position.x() + (rand_int - 60) * 0.1;
    pt.y() = odom_position.y() + (rand_int - 60) * 0.1;
} else {
    // 90% chance: randomly pick from topo_sample_map
    pt = global_map->topo_sample_map[random_index];
}
```

For local planning (`createLocalGraph`), `getLocalSample()` constrains sampling to an
ellipsoidal region between start and end:
- Semi-axis along start→end: `0.5 * dist + 2.0 m`
- Semi-axis perpendicular: `7.0 m`

### 1.3 Guard Creation Rules for Random Samples

Each random sample `pt` is checked against existing guards via `findVisibGuard()`:

| `visib_guards.size()` | Action | Rationale |
|------------------------|--------|-----------|
| **0** (invisible to all) | **Create Guard** | New region unreachable from existing graph |
| **1** (sees exactly 1 guard) | Check obstacle density | May need guard if in narrow passage |
| **2+** (sees 2+ guards) | **Create Connector** (see §2) | Can bridge existing guards |

**Single-visibility heuristic** (1 guard visible):

```cpp
// createGraph(), line ~292
if (visib_guards.size() == 1) {
    if (exist_unvisiual > 2) continue;   // too many blocked guards → skip
    if (min_distance < MIN_GUARD_SPACING) continue; // too close → skip (0.35m)

    // Count obstacles within 0.5 m radius (40 ray samples)
    for (int i = 0; i < 40; i++) {
        edge_x = start_x + sin(i * π/20) * 0.5;
        edge_y = start_y + cos(i * π/20) * 0.5;
        if (isOccupied(edge_x, edge_y)) obs_num++;
    }
    if (obs_num >= 10) {
        // Dense obstacles around this point → narrow passage → needs guard
        m_graph.push_back(new GraphNode(pt, Guard, ++node_id));
    }
}
```

This heuristic adds guards in narrow passages where a point can see one guard but is
surrounded by obstacles (≥25% of the 40 radial probes hit walls).  The local variant uses a
threshold of 20 (50%).

---

## 2. Connection Point Generation

Connection points (connectors) are created when a sample sees **exactly 2 guard nodes**
that are not already connected.

### 2.1 Visibility Check: `findVisibGuard()`

```cpp
// findVisibGuard(), line ~799
for (auto iter = m_graph.begin(); iter != m_graph.end(); ++iter) {
    if ((*iter)->type_ == Connector) continue;   // skip connectors
    if (distance > 5.0) continue;                // max visibility range

    if (lineVisib(pt, (*iter)->pos, 0.2, pt_temp, 0)) {
        visib_guards.push_back(*iter);
        if (visib_num == 2) {
            if (needConnection(visib_guards[0], visib_guards[1], pt)) {
                break;  // found 2 unconnected guards → create connector
            } else {
                // Already connected → discard second, keep searching
                visib_guards[0] = visib_guards[1];
                visib_guards.pop_back();
                exist_unvisiual++;
            }
        }
    }
}
```

Key constraints:
- Only Guard-type nodes are considered (connectors are skipped)
- Maximum visibility radius: **5.0 m**
- Visibility checked via `lineVisib()` at **0.2 m** step resolution

### 2.2 Connection Necessity: `needConnection()`

Two guards need a connection if they **share no common neighbor**:

```cpp
// needConnection(), line ~853
bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt) {
    for (auto& n1 : g1->neighbors)
        for (auto& n2 : g2->neighbors)
            if (n1->m_id == n2->m_id) return false;  // already linked
    return true;
}
```

This prevents redundant connectors — if guards g1 and g2 already have a shared neighbor
(another connector), adding another connection is unnecessary.

### 2.3 Height-Aware Connection: `checkHeightFeasible()`

When two guards are identified for connection, the connector is created with
height-dependent edge directionality:

```cpp
// checkHeightFeasible(), line ~551
GraphNode::Ptr connector = new GraphNode(pt, Connector, ++node_id);

if (heightFeasible(g1->pos, pt, direction)) {
    // Bidirectional: g1 ↔ connector
    g1->neighbors.push_back(connector);
    connector->neighbors.push_back(g1);
} else {
    if (direction == 1) {
        // Uphill only: g1 → connector (can go up but not down)
        g1->neighbors.push_back(connector);
        connector->neighbors_but_noconnected.push_back(g1);
    } else if (direction == 2) {
        // Downhill only: connector → g1
        g1->neighbors_but_noconnected.push_back(connector);
        connector->neighbors.push_back(g1);
    }
}
// Same logic for g2
```

**`heightFeasible()`** walks from p2 (connector) to p1 (guard) at 0.05 m steps, checking:
- Height difference between consecutive samples > **0.3 m** → infeasible (cliff/wall)
- Height difference > **0.10 m** → unidirectional (direction=1: uphill, direction=2: downhill)
- Both endpoints in bridge-tunnel areas with > 0.3 m z-difference → infeasible
- Bridge-tunnel cells use `last_height` interpolation instead of BEV height

The `neighbors_but_noconnected` list stores one-way edges — the Dijkstra search does NOT
traverse these, but they serve as metadata for visualization and future path analysis.

---

## 3. Line-of-Sight Visibility: `lineVisib()`

The TopoSearcher has its own `lineVisib()` (separate from `AstarPathFinder::lineVisib()`):

```cpp
// TopoSearch::lineVisib(), line ~921
bool lineVisib(p1, p2, thresh=0.2, pc, caster_id=0) {
    int n = distance(p1, p2) / thresh;
    for (int i = 0; i <= n; i++) {
        ray_pt = p2 + i * thresh * (p1 - p2) / distance;
        // 1. Check height discontinuity > 0.4 m → return false
        // 2. Check isOccupied (with bridge-tunnel handling) → return false
    }
    return true;
}
```

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `thresh` | 0.2 m | Ray-march step size |
| Height limit | 0.4 m | Max height change between consecutive samples |
| `caster_id` | 0 (guard sampling), 1 (path comparison) | Controls distance threshold behavior |
| Close-range check | dist < 0.4 m AND Δz > 0.4 m → false | Prevents visibility across height gaps |

Note: This is **different** from `AstarPathFinder::lineVisib()` which has `cell_margin`
support (Fix 47a) and finer height gradient checks (+0.12/−0.3 m thresholds).

---

## 4. Graph Search: Dijkstra

After the graph is built, `DijkstraSearch()` finds the shortest path:

```cpp
// DijkstraSearch(), line ~440
// Standard O(V²) Dijkstra from node 0 (start) to node_id (default 1 = end)
// Edge weights: Euclidean distance between connected nodes
// Result stored in min_path (reversed start→end order)
```

If the end node is unreachable (`minDist[node_id] > 10000`), the path is empty and
fallback mechanisms activate (see §5).

---

## 5. Fallback Mechanisms

### 5.1 Force-Connect (Post-Sampling)

After sampling completes, if start or end nodes have **no neighbors**:

```cpp
// createGraph(), line ~334
for (ti in {0, 1}) {  // 0=start, 1=end
    if (m_graph[ti]->neighbors.empty()) {
        // Find nearest visible guard and force-connect
        for (g in all_guards) {
            if (lineVisib(terminal, guard, 0.2, pc, 0)) {
                if (distance < best_dist) { best_dist = d; best_g = g; }
            }
        }
        m_graph[ti] ↔ m_graph[best_g];  // bidirectional edge
    }
}
```

Log message: `[Topo] Force-connected start/end node to guard N (dist=X.XXm)`

### 5.2 BFS Connectivity Retry (Post-Dijkstra)

If Dijkstra finds no path (end unreachable), BFS from start identifies all reachable nodes:

```cpp
// createGraph(), line ~355
if (min_path.empty()) {
    BFS from node 0 → reachable[] array
    if (!reachable[1]) {
        // End is disconnected → force-connect to nearest reachable visible guard
        for (g in reachable_guards) {
            if (lineVisib(end, guard, 0.2, pc, 0) && distance < best_dist) {
                best_g = g;
            }
        }
        m_graph[1] ↔ m_graph[best_g];
        searchPaths();  // retry Dijkstra
    }
}
```

Log message: `[Topo] Force-connected end to reachable node N (dist=X.XXm)`

These fallbacks ensure the topo search almost always produces a path, even if the
sampling didn't create a fully connected graph.

---

## 6. Post-Search Path Smoothing: `smoothTopoPath()`

After Dijkstra produces `min_path`, the raw path is smoothed by `AstarPathFinder::smoothTopoPath()`:

1. **Greedy line-of-sight pruning**: Walks along the topo path and attempts direct
   shortcuts to the current head position. If the shortcut has clear line-of-sight (via
   `AstarPathFinder::lineVisib()` with `cell_margin=3`), intermediate waypoints are pruned.

2. **Collision detection**: When a shortcut fails (obstacle in the way), `getNearPoint()`
   nudges the collision point perpendicular to the path to find a nearby clear waypoint.

3. **Fix 47a/b**: Added safety margin (`cell_margin=3` = 0.15 m) to prevent shortcuts
   that skim too close to obstacles, and improved iteration-limit fallback to return
   partial smooth + raw tail instead of the entire raw topo path.

---

## 7. Typical Graph Statistics (from logs)

| Metric | Typical Value |
|--------|---------------|
| Pre-sampling guards (from keypoints) | 200–230 |
| Post-sampling graph size | 680–750 nodes |
| Dijkstra time | 1–2 ms |
| Total topo search time | 30–70 ms |
| Force-connect distance | 1–3 m |

---

## 8. Visualization Topics

| Topic | Content |
|-------|---------|
| `/trajectory_generation/topo_point_guard` | Guard node positions (red markers) |
| `/trajectory_generation/topo_point_connection` | Connection node positions (blue markers) |
| `/trajectory_generation/topo_line` | Graph edges between nodes |
| `/trajectory_generation/topo_point_path` | Dijkstra shortest path nodes |
| `/trajectory_generation/astar_path_vis` | Smoothed topo path (after `smoothTopoPath`) |
