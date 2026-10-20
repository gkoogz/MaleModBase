import bpy
from pathlib import Path
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "work" / "generic-male-inspection"
OUT.mkdir(parents=True, exist_ok=True)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.gltf(filepath=str(ROOT / "assets" / "generic-male" / "reference.glb"))
for obj in list(bpy.context.scene.objects):
    if obj.type not in {"MESH", "ARMATURE"}:
        bpy.data.objects.remove(obj, do_unlink=True)
body = next(obj for obj in bpy.context.scene.objects if obj.type == "MESH" and len(obj.data.vertices) > 1000)
body.hide_render = False
for polygon in body.data.polygons:
    polygon.use_smooth = True
bpy.ops.mesh.primitive_plane_add(size=5, location=(0, 0, 0))
floor = bpy.context.object
mat = bpy.data.materials.new("Floor")
mat.diffuse_color = (0.04, 0.05, 0.07, 1)
floor.data.materials.append(mat)

def look_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()

bpy.ops.object.camera_add()
camera = bpy.context.object
camera.data.lens = 58
bpy.context.scene.camera = camera
for loc in ((3, -5, 2.2), (0, -5.5, 1.2), (0, 5.5, 1.2)):
    data = bpy.data.lights.new("Area", "AREA")
    data.energy = 900
    data.shape = "DISK"
    data.size = 3
    light = bpy.data.objects.new("Area", data)
    bpy.context.collection.objects.link(light)
    light.location = loc
    look_at(light, (0, 0, 0.9))
scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x = 900
scene.render.resolution_y = 1200
scene.render.resolution_percentage = 100
scene.world.color = (0.025, 0.03, 0.04)
for name, loc in {"front": (0, -4.0, 1.2), "rear": (0, 4.0, 1.2), "three-quarter": (2.8, -3.8, 1.35)}.items():
    camera.location = loc
    look_at(camera, (0, 0, 0.95))
    scene.render.filepath = str(OUT / f"{name}.png")
    bpy.ops.render.render(write_still=True)
