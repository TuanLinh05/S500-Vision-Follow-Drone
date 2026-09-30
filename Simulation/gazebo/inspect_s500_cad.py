from pathlib import Path

import cadquery as cq

source = Path("/mnt/d/Intern_Data/S500 Drone/3D model/s500-frame-1.snapshot.6/S500 Frame.stp")
assembly = cq.importers.importStep(str(source)).val()
overall = assembly.BoundingBox()

print(f"source={source}")
print(
    "overall_mm="
    f"{overall.xlen:.3f} x {overall.ylen:.3f} x {overall.zlen:.3f}; "
    f"center=({overall.center.x:.3f}, {overall.center.y:.3f}, {overall.center.z:.3f})"
)

solids = assembly.Solids()
print(f"solids={len(solids)}")
for index, solid in enumerate(solids):
    box = solid.BoundingBox()
    print(
        f"solid[{index}] "
        f"size=({box.xlen:.3f}, {box.ylen:.3f}, {box.zlen:.3f}) "
        f"center=({box.center.x:.3f}, {box.center.y:.3f}, {box.center.z:.3f})"
    )
