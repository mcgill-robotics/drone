import bpy
import numpy as np
import os
from mathutils import Vector, Euler, Matrix


class ObjectBB:
    def __init__(self, obj, bounding_box):
        self.obj = obj
        self.bb: BB = bounding_box


class AABB:
    def __init__(self, min_x, min_y, min_z, max_x, max_y, max_z):
        self.min_x = min_x
        self.min_y = min_y
        self.min_z = min_z
        self.max_x = max_x
        self.max_y = max_y
        self.max_z = max_z

    def intersects1D(self, min1, max1, min2, max2):
        return max1 >= min2 and max2 >= min1

    def intersects(self, other_aabb):
        return (self.intersects1D(self.min_x, self.max_x,
                                  other_aabb.min_x, other_aabb.max_x)
                and self.intersects1D(self.min_y, self.max_y,
                                      other_aabb.min_y, other_aabb.max_y))

    def __str__(self):
        return f"(({self.min_x}, {self.min_y}), ({self.max_x}, {self.max_y}))"

    def __repr__(self):
        return str(self)


class BB:
    def __init__(self, bb_min_coords, bb_max_coords):
        self.min_x, self.min_y, self.min_z = bb_min_coords
        self.max_x, self.max_y, self.max_z = bb_max_coords

    @staticmethod
    def get_mesh_bb(obj):
        update_scene()
        bb_min, bb_max = BB.transform_adjusted_mesh(obj)
        return BB(bb_min, bb_max)

    @staticmethod
    def transform_adjusted_mesh(obj):
        # TODO: Do everything from world matrix, then undo world matrix at the very end
        def transform_adjusted_mesh_helper(obj):
            bb_min = [float("inf")] * 3
            bb_max = [float("-inf")] * 3
            if obj.type == "MESH":
                bbx = [obj.matrix_world @
                       Vector(bbx) for bbx in obj.bound_box]
                for bb in bbx:
                    for dim in range(3):
                        bb_min[dim] = min(bb_min[dim], bb[dim])
                        bb_max[dim] = max(bb_max[dim], bb[dim])

            for child in obj.children:
                child_bb = transform_adjusted_mesh_helper(child)
                child_bb_min, child_bb_max = child_bb
                for dim in range(3):
                    bb_min[dim] = min(
                        bb_min[dim], child_bb_min[dim])
                    bb_max[dim] = max(
                        bb_max[dim], child_bb_max[dim])
            return bb_min, bb_max
        root_transform_inverse = obj.matrix_world.copy()
        root_transform_inverse.invert()
        world_bb_min, world_bb_max = transform_adjusted_mesh_helper(obj)
        return (root_transform_inverse @ Vector(world_bb_min), root_transform_inverse @ Vector(world_bb_max))

    def get_aabb(self, world_matrix) -> AABB:

        pts = []
        for x in [self.min_x, self.max_x]:
            for y in [self.min_y, self.max_y]:
                for z in [self.min_z, self.max_z]:
                    pt = [x, y, z]
                    pts.append(pt)
        pts_trans = [world_matrix @ Vector(pt) for pt in pts]
        min_p = np.array(pts_trans).min(axis=0)
        max_p = np.array(pts_trans).max(axis=0)
        return AABB(min_p[0], min_p[1], min_p[2], max_p[0], max_p[1], max_p[2])

    def __str__(self):
        return f"(({self.min_x}, {self.min_y}, {self.min_z}), ({self.max_x}, {self.max_y}, {self.max_z}))"

    def __repr__(self):
        return str(self)


def draw_cube_wireframe(p_min, p_max):
    center = (np.array(p_max) + np.array(p_min)) / 2
    scale = (np.array(p_max) - np.array(p_min)) / 2
    bpy.ops.mesh.primitive_cube_add(location=center, scale=scale)
    bpy.ops.object.modifier_add(type="WIREFRAME")


def update_scene():
    bpy.context.scene.frame_set(
        bpy.context.scene.frame_current + 1
    )


def setup_camera(height=30, lens=16, sensor_width=7):
    """
        Returns how much of the ground can be see (the width the of the
        rectangle of the ground in view)
    """
    cam = bpy.context.scene.camera
    cam.rotation_euler = [0, 0, 0]
    cam.location = [0, 0, height]
    cam = bpy.data.cameras["Camera"]
    cam.lens = lens
    cam.sensor_width = sensor_width
    return (((height * 1000.) * (sensor_width)) / lens) / 1000.


def setup_world_light():
    world_node_tree = bpy.context.scene.world.node_tree
    world_node_tree.nodes.clear()

    world_background_node = world_node_tree.nodes.new(
        type="ShaderNodeBackground")
    world_output_node = world_node_tree.nodes.new(
        type="ShaderNodeOutputWorld")
    world_node_tree.links.new(
        world_background_node.outputs["Background"],
        world_output_node.inputs["Surface"])
    world_background_node.inputs[1].default_value = 1.6


def setup_light():
    light = bpy.data.lights["Light"]
    light.type = "SUN"
    light.energy = 4.
    light = bpy.data.objects["Light"]
    light.location = [0., 0.0, 1000.]
    light.rotation_euler = [0., 0.0, 0.0]
    return light


def setup_ground():
    bpy.ops.import_scene.gltf(filepath="./Models/Ground/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["asphalt"]
    obj.location.z = -0.6
    return obj


def setup_car():
    bpy.ops.import_scene.gltf(filepath="./Models/Car/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["car"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_mannequin():
    bpy.ops.import_scene.gltf(filepath="./Models/Mannequin/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["mannequin"]
    obj.location = [0, 0, -1.46]
    obj.scale = [1.0, 1.0, 1.0]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_motorcycle():
    bpy.ops.import_scene.gltf(filepath="./Models/Motorcycle/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["motorcycle"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_airplane():
    bpy.ops.import_scene.gltf(filepath="./Models/Airplane/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["airplane"]
    obj.scale.x = 0.2
    obj.scale.y = 0.2
    obj.scale.z = 0.2
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_bus():
    bpy.ops.import_scene.gltf(filepath="./Models/Bus/scene.gltf",
                              merge_vertices=True)

    obj = bpy.data.objects["bus"]
    obj.scale.x = 0.5
    obj.scale.y = 0.5
    obj.scale.z = 0.5
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_boat():
    bpy.ops.import_scene.gltf(filepath="./Models/Boat/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["boat"]
    obj.scale.x = 0.01
    obj.scale.y = 0.01
    obj.scale.z = 0.01
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_stop_sign():
    bpy.ops.import_scene.gltf(filepath="./Models/Stop/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["stop sign"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_snowboard():
    bpy.ops.import_scene.gltf(filepath="./Models/Snowboard/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["snowboard"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_umbrella():
    bpy.ops.import_scene.gltf(filepath="./Models/Umbrella/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["umbrella"]
    obj.scale = [0.5, 0.5, 0.5]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_soccer():
    bpy.ops.import_scene.gltf(filepath="./Models/Soccer/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["soccer"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_baseball_bat():
    bpy.ops.import_scene.gltf(filepath="./Models/Baseball/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["baseball bat"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_mattress():
    bpy.ops.import_scene.gltf(filepath="./Models/Mattress/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["mattress"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_tennis_racket():
    bpy.ops.import_scene.gltf(filepath="./Models/Tennis/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["tennis racket"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_suitcase():
    bpy.ops.import_scene.gltf(filepath="./Models/Suitcase/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["suitcase"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))


def setup_skis():
    bpy.ops.import_scene.gltf(filepath="./Models/Skis/scene.gltf",
                              merge_vertices=True)
    obj = bpy.data.objects["skis"]
    return ObjectBB(obj, BB.get_mesh_bb(obj))
