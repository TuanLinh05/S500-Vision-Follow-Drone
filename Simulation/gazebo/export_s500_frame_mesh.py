"""Export the supplied S500 CAD assembly to a Gazebo visual mesh."""

from pathlib import Path

import cadquery as cq

workspace = Path("/mnt/d/Intern_Data/S500 Drone")
source = workspace / "3D model/s500-frame-1.snapshot.6/S500 Frame.stp"
mesh_dir = workspace / "Simulation/gazebo/models/s500_quad_x/meshes"
mesh_dir.mkdir(parents=True, exist_ok=True)
full_output = mesh_dir / "s500_frame_full.stl"

assembly = cq.importers.importStep(str(source)).val()
cq.exporters.export(assembly, str(full_output), tolerance=0.10, angularTolerance=0.20)

full_bounds = assembly.BoundingBox()
print(f"exported={full_output}")
print(f"full_size_mm={full_bounds.xlen:.3f} x {full_bounds.ylen:.3f} x {full_bounds.zlen:.3f}")
