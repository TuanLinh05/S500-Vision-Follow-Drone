# S500 Gazebo model

The Gazebo visual model uses the supplied `S500 Frame.stp` CAD assembly,
including its original curved landing gear.  The rigid-body collision shape
remains deliberately simple so PX4 SITL physics stays stable and tunable.

## Current assumptions

- 500 mm diagonal motor spacing in an X configuration;
- DJI Phantom 9450 propeller diameter represented as 0.240 m;
- 1.68 kg all-up mass estimate;
- DJI 940 kV + 4S propulsion parameters are provisional, not validated;
- the CAD visual mesh is generated at a 0.001 scale from
  `3D model/s500-frame-1.snapshot.6/S500 Frame.stp` with
  `export_s500_frame_mesh.py`;
- the MTF-01 model has a 50 Hz downward optical-flow camera and 50 Hz,
  0.10–8 m downward rangefinder, mapped to PX4's native Gazebo bridge;
- Pixhawk 6C and GPS M10 are represented visually; IMU, GPS, barometer and
  magnetometer are declared in SDF and named to match PX4's Gazebo bridge.

## Run in WSL

```bash
bash Simulation/gazebo/run_s500_gazebo_wsl.sh
```

The launcher uses OGRE1 by default on this WSLg setup because OGRE2 creates a
blank viewport here. OGRE1 still uses hardware acceleration:

```bash
env -u LIBGL_ALWAYS_SOFTWARE S500_GZ_RENDER_ENGINE=ogre \
  bash Simulation/gazebo/run_s500_gazebo_wsl.sh
```

WSLg reports the Intel Iris Xe / D3D12 renderer as hardware accelerated; do
not set `LIBGL_ALWAYS_SOFTWARE=1`. OGRE2 can still be tested explicitly with
`S500_GZ_RENDER_ENGINE=ogre2` on a setup where its viewport renders correctly.

The Gazebo launcher also finds PX4's `libOpticalFlowSystem.so` in the default
PX4 build directory. Set `PX4_AUTOPILOT_HOME` if PX4 is elsewhere. The dark
checkerboard target under the vehicle is intentional: it gives the simulated
downward camera visual features while testing low-altitude position hold.

The vehicle is deliberately spawned disarmed and receives no motor command in
this standalone Gazebo world.

## PX4 SITL

The S500 is configured to attach PX4's Gazebo bridge to the existing model
`s500_quad_x`. Build PX4 v1.17 (or a compatible release) in the Linux WSL
filesystem, then run the Gazebo and PX4 launchers in separate terminals:

```bash
bash Simulation/gazebo/run_s500_gazebo_wsl.sh
bash Simulation/gazebo/run_s500_px4_sitl_wsl.sh
```

The PX4 launcher starts airframe `4001` in standalone mode and binds it to the
existing S500 model; it does not spawn a second vehicle. It remains disarmed.
It also enables `MAV_0_BROADCAST=1` so the Windows-host QGroundControl link can
receive MAVLink packets from WSL2.

### QGroundControl on Windows

In WSL2 NAT mode, QGroundControl for Windows uses a UDP Comm Link with local
port `14550` and server address `<WSL-IP>:18570` (for SITL instance 0).
The WSL IP can change after a WSL restart; obtain it with:

```bash
ip -4 addr show eth0
```

Do not arm until the motor thrust coefficient, total mass/CG and control
allocation geometry have been calibrated against the physical S500.
