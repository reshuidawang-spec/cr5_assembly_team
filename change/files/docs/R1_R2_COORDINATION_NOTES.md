# R1/R2 Coordination Notes

## Deferred Check After R2 Safe-Wait Validation

- After R1 places the box, R2 should place the PCB immediately.
- While R2 is placing the PCB, R1 may also need to move to a safe intermediate
  waypoint instead of remaining near the assembly/interference region.
- This second-stage R1 safe waypoint is deferred until the current R2
  post-pick safe-wait task is validated in RViz and CoppeliaSim.
