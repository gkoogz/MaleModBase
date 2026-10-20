from pathlib import Path
import bpy
from mathutils import Vector
import numpy as np

root = Path(__file__).resolve().parents[1]
out = root / "assets" / "speculative-hypertrophy-doll" / "reference-inspection"
out.mkdir(parents=True, exist_ok=True)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.wm.obj_import(filepath=str(root / "assets" / "wolverine-reference" / "final-reference.obj"))
obj = bpy.context.selected_objects[0]
obj.name = "Wolverine_Final_Reference"
mat = bpy.data.materials.new("Inspection")
mat.diffuse_color = (0.55, 0.25, 0.16, 1.0)
obj.data.materials.append(mat)
coordinates = np.asarray([tuple(vertex.co) for vertex in obj.data.vertices], dtype=np.float64)
centered = coordinates - coordinates.mean(axis=0)
_, eigenvectors = np.linalg.eigh(np.cov(centered, rowvar=False))
basis = eigenvectors[:, ::-1]
if basis[0, 0] > 0.0:
    basis[:, 0] *= -1.0
local = centered @ basis
axial = local[:, 0]
u = (axial - axial.min()) / (axial.max() - axial.min())
transformed = np.empty_like(local)
transformed[:, 0] = local[:, 2] * 0.0107
transformed[:, 1] = -0.18 + local[:, 1] * 0.0105 - 0.115 * np.power(1.0 - u, 1.35)
transformed[:, 2] = 0.27 + u * 0.77
for vertex, coordinate in zip(obj.data.vertices, transformed):
    vertex.co = coordinate
for poly in obj.data.polygons:
    poly.use_smooth = True

scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x = 900
scene.render.resolution_y = 900
scene.render.resolution_percentage = 100
scene.world.color = (0.025, 0.025, 0.03)
bpy.ops.object.light_add(type="POINT", location=(3, -3, 4))
bpy.context.object.data.energy = 1200
bpy.context.object.data.shadow_soft_size = 4
bpy.ops.object.camera_add()
camera = bpy.context.object
camera.data.type = "ORTHO"
camera.data.ortho_scale = 1.4
scene.camera = camera

def look_at(location):
    camera.location = location
    camera.rotation_euler = (Vector((0, 0, 0)) - camera.location).to_track_quat("-Z", "Y").to_euler()

for name, location in {
    "axis_x.png": (4, -0.18, 0.65),
    "axis_y.png": (0, -4, 0.65),
    "axis_z.png": (0, -0.18, 4),
}.items():
    camera.location = location
    camera.rotation_euler = (Vector((0, -0.18, 0.65)) - camera.location).to_track_quat("-Z", "Y").to_euler()
    scene.render.filepath = str(out / name)
    bpy.ops.render.render(write_still=True)
