# Isaac Sim patient cart with OpenXR

This configuration starts the three-PSM/ECM Isaac Sim cart, the dVRK OpenXR
console, and RTSP stereo video. The scene is fixed to
`ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml`; its camera is consumed directly by
`sawOpenXR`, so no local video overlay process is required.

Build and source `dvrk_isaac_sim`, `sawOpenXR`, and the dVRK ROS packages, then
run:

```bash
ros2 launch dvrk_isaac_sim open_xr.launch.py
```

The launch uses the machine-specific `share/isaac_sim.yaml` created during the
package build with `ISAAC_SIM_DIR` set, and forces headless mode. Override it
with `config:=/path/to/isaac_sim.yaml` when required. The only other launch
argument is `console:=...`.