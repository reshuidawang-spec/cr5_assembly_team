# HeuristicGuided Post-Fix Formal Rerun Analysis (2026-03-24)

## 1. Run Context

This note summarizes the post-fix formal reruns of `HeuristicGuided` after the B+ guidance logic was updated to:

- difficulty-conditioned ellipsoid mixture
- environment-aware anchor guides
- hard-scene guide-first triggering
- post-direct guide suppression on non-hard scenes

Formal rerun files:

- `simple`
  - results: `test_results/benchmarks/simple/raw/20260324_081713_711_planner_comparison_simple_results.csv`
  - summary: `test_results/benchmarks/simple/raw/20260324_081713_711_planner_comparison_simple_summary.csv`
- `v2`
  - results: `test_results/benchmarks/v2/raw/20260324_081904_937_planner_comparison_v2_results.csv`
  - summary: `test_results/benchmarks/v2/raw/20260324_081904_937_planner_comparison_v2_summary.csv`

These runs should be treated as the current paper-facing `HeuristicGuided` numbers.

## 2. Headline Findings

The most important conclusions are:

1. The method is no longer only validated on hard-scene subsets. It now has full-scene 10-repeat reruns on both `simple` and `v2`.
2. The selective behavior is stable:
   - `hard / extreme` scenes use `active_guidance`
   - `easy / medium` scenes remain `direct-only`
3. The main remaining latency issue is no longer false-positive guidance. It is the occasional slow `direct` solve on easier scenes.

## 3. Paper-Facing Numbers

### 3.1 Simple

- success: `100.0%` (`60/60`)
- mean: `425.5 ms`
- median: `240.5 ms`
- budget hit: `0.0%`
- fast solve: `80.0%`

Relative to the strongest classical baseline currently kept in the paper (`LBTRRT`):

- mean planning time reduction: `83.8%`
- budget hit reduction: `23.3% -> 0.0%`

Relative to the old `HeuristicGuided` formal result:

- mean planning time: `845.8 -> 425.5 ms`
- reduction: `49.7%`

### 3.2 V2

- success: `100.0%` (`40/40`)
- mean: `302.0 ms`
- median: `125.5 ms`
- budget hit: `0.0%`
- fast solve: `85.0%`

Relative to the strongest classical baseline currently kept in the paper (`BFMT`):

- mean planning time reduction: `88.2%`
- budget hit reduction: `25.0% -> 0.0%`

Relative to the old `HeuristicGuided` formal result:

- mean planning time: `377.1 -> 302.0 ms`
- reduction: `19.9%`

## 4. Gate Behavior

The current selective pattern is now stable across both benchmarks.

### 4.1 Simple

- `Easy_TopCenter`: `direct-only`
- `Medium_SideSurface`: `direct-only`
- `MediumPlus_RightUpperAngled`: `direct-only`
- `Hard_HoleShallow`: `active_guidance`
- `HardPlus_HoleEdgeOffset`: `active_guidance`
- `Extreme_HoleDeep`: `active_guidance`

### 4.2 V2

- `Easy_HoleCenter`: `direct-only`
- `Medium_HoleEdge`: `direct-only`
- `Hard_DeepInterior`: `active_guidance`
- `Extreme_NarrowPassage`: `active_guidance`

This is the current evidence that the method should be described as `difficulty-selective active guidance with direct preservation`, not as a globally forced guidance policy.

## 5. Residual Risk

The main residual issue is the long-tail behavior of `direct` on some easy / medium scenes:

- `simple`
  - `Easy_TopCenter` direct slow rate: `30%`
  - `Medium_SideSurface` direct slow rate: `30%`
  - `MediumPlus_RightUpperAngled` direct slow rate: `60%`
- `v2`
  - `Easy_HoleCenter` direct slow rate: `30%`
  - `Medium_HoleEdge` direct slow rate: `30%`

This should be treated as a secondary enhancement line:

- `slow-direct rescue`

It should not change the current main paper claim, because the current B+ contribution is already supported by stable full-scene reruns.

## 6. Recommended Paper Wording

Recommended main claim:

> The proposed guidance layer activates selectively on difficult contact-measurement scenes while preserving direct planning on easier scenes, yielding zero budget hits and substantially lower mean planning time than the strongest classical baselines in both controlled and STL-based benchmarks.

Recommended interpretation:

- The contribution is not merely a tuned direct-first baseline.
- The contribution is a selective guidance policy that now demonstrably changes routing behavior on difficult scenes.
- The main value is robust suppression of long-tail budget-hit behavior across constrained geometries.
