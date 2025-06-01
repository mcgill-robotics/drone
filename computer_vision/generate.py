import numpy as np
import bpy_extras
import importlib
from math import pi
import bpy
from mathutils import Vector
import os
import sys
# import helpers.setup_helpers as helpers
dir_name = os.path.dirname(__file__)
sys.path.insert(1, dir_name)
helpers = importlib.import_module("helpers.setup_helpers")
dataset_dir = os.path.join(dir_name, "dataset")
imgs_dir_post_fix = "images"
labels_dir_post_fix = "labels"


CAR = "car"
MANNEQUIN = "mannequin"
MOTORCYCLE = "motorcycle"
AIRPLANE = "airplane"
BUS = "bus"
BOAT = "boat"
STOP_SIGN = "stop sign"
SNOWBOARD = "snowboard"
UMBRELLA = "umbrella"
SOCCER = "soccer"
BASEBALL_BAT = "baseball bat"
MATTRESS = "mattress"
TENIS_RACKET = "tennis racket"
SUITCASE = "suitcase"
SKIS = "skis"
OBJECTS_LABEL = [CAR, MANNEQUIN, MOTORCYCLE, AIRPLANE, BUS, BOAT, STOP_SIGN,
                 SNOWBOARD, UMBRELLA, SOCCER, BASEBALL_BAT, MATTRESS,
                 TENIS_RACKET, SUITCASE, SKIS]
MAX_NUM_OBJECTS = len(OBJECTS_LABEL)
# [train, val, test]
NUM_INSTANCES_TO_GENERATE = [1, 1, 1]
MAX_NUM_ATTEMPTS = 5


def generate_label(root_dir, instance_index, objs_and_aabbs):
    labels_file = os.path.join(
        root_dir, labels_dir_post_fix, f"{instance_index:05d}.txt")
    with open(labels_file, "w") as f:
        for obj, bbox in objs_and_aabbs:
            string = f"{obj} {bbox[0]:.6f} {bbox[1]:.6f} {bbox[2]:.6f} {bbox[3]:.6f}\n"
            f.write(string
                    )


def get_world_aabb(obj, node_tree=None):
    helpers.update_scene()
    matrix_world = obj.obj.matrix_world
    return obj.bb.get_aabb(matrix_world)


def generate_image(objects, ground_width, ground_height=None):
    """
    Assumes a square with with ground with into which a number of objects will
    go, will then render the image and return the objects label and their world
    binding boxes
        [[label, [[min_x, min_y], [max_x, max_y]]], ...]

        """
    if (ground_height is None):
        ground_height = 3 / 7 * ground_width
    labels_used = []
    bbs_used = []
    objects_to_choose_from = list(np.copy(OBJECTS_LABEL))
    for i in range(MAX_NUM_OBJECTS):
        random_index = np.random.choice(range(len(objects_to_choose_from)))
        obj_label = OBJECTS_LABEL.index(objects_to_choose_from[random_index])
        chosen_obj = objects[objects_to_choose_from[random_index]]
        objects_to_choose_from.pop(random_index)
        for attempt in range(MAX_NUM_ATTEMPTS):
            rand_x = np.random.random() * ground_width - (ground_width / 2)
            rand_y = np.random.random() * ground_height - (ground_height / 2)

            rand_heading = np.random.random() * pi * 2
            chosen_obj.obj.location = [rand_x, rand_y, 0.]
            # chosen_obj.rotation_euler = Euler((0, 0, rand_heading), 'XYZ')
            chosen_obj.obj.rotation_mode = "XYZ"
            chosen_obj.obj.rotation_euler.z = rand_heading

            aabb = get_world_aabb(chosen_obj)

            if any(map(lambda used: aabb.intersects(used), bbs_used)):
                put_away(chosen_obj.obj)
                continue
            else:
                # helpers.draw_cube_wireframe([aabb.min_x, aabb.min_y, 0.], [
                #                             aabb.max_x, aabb.max_y, 1.])
                labels_used.append(obj_label)
                bbs_used.append(aabb)
                break

    bpy.ops.render.render(write_still=True)
    return list(zip(labels_used, bbs_used))


def put_away(obj):
    obj.location = [1000, 1000, 1000]


def map_to_camera_bbs(objs_and_bbs):
    scene = bpy.context.scene
    camera = scene.camera
    new_boxes = []
    labels = []
    for label, aabb in objs_and_bbs:
        labels.append(label)
        bbox = [Vector([x, y, z]) for x in [aabb.min_x, aabb.max_x]
                for y in [aabb.min_y, aabb.max_y] for z in [aabb.min_z, aabb.max_z]]
        coords_2d = [bpy_extras.object_utils.world_to_camera_view(
            scene, camera, corner) for corner in bbox]

        min_x = max(min(pt.x for pt in coords_2d), 0)
        max_x = min(max(pt.x for pt in coords_2d), 1)
        min_y = max(min(pt.y for pt in coords_2d), 0)
        max_y = min(max(pt.y for pt in coords_2d), 1)

        x_center = (min_x + max_x) / 2
        y_center = 1 - (min_y + max_y) / 2  # flip Y
        width = max_x - min_x
        height = max_y - min_y
        new_boxes.append((x_center, y_center, width, height))

    return list(zip(labels, new_boxes))


def setup_data_yml(objects):
    path = os.path.join(dataset_dir, "data.yaml")
    with open(path, "w") as fp:
        fp.write(f"train: ../train/images\n")
        fp.write(f"val: ../val/images\n")
        fp.write(f"test: ../test/images\n")
        fp.write(f"nc: {len(OBJECTS_LABEL)}\n")
        fp.write(f"names: {OBJECTS_LABEL}\n")


def main():
    # Setup Camera
    ground_view_width = helpers.setup_camera()
    helpers.setup_ground()
    helpers.setup_light()
    helpers.setup_world_light()
    # Setup Objects
    objects: dict[str, helpers.ObjectBB] = {
        CAR: helpers.setup_car(),
        MANNEQUIN: helpers.setup_mannequin(),
        MOTORCYCLE: helpers.setup_motorcycle(),
        AIRPLANE: helpers.setup_airplane(),
        BUS: helpers.setup_bus(),
        BOAT: helpers.setup_boat(),
        STOP_SIGN: helpers.setup_stop_sign(),
        SNOWBOARD: helpers.setup_snowboard(),
        UMBRELLA: helpers.setup_umbrella(),
        SOCCER: helpers.setup_soccer(),
        BASEBALL_BAT: helpers.setup_baseball_bat(),
        MATTRESS: helpers.setup_mattress(),
        TENIS_RACKET: helpers.setup_tennis_racket(),
        SUITCASE: helpers.setup_suitcase(),
        SKIS: helpers.setup_skis()
    }
    os.makedirs(dataset_dir,
                exist_ok=True)
    setup_data_yml(objects)
    scene = bpy.context.scene
    bpy.data.objects.remove(bpy.data.objects["Cube"])
    for set_portion_index, num_samples in enumerate(NUM_INSTANCES_TO_GENERATE):
        portion = "train" if set_portion_index == 0 else "val" if set_portion_index == 1 else "test"
        os.makedirs(os.path.join(dataset_dir, portion,
                    imgs_dir_post_fix), exist_ok=True)
        os.makedirs(os.path.join(dataset_dir, portion,
                    labels_dir_post_fix), exist_ok=True)
        for i in range(1, num_samples+1):
            # Reset all objects
            for key in objects:
                put_away(objects[key].obj)

            scene.render.filepath = os.path.join(
                dataset_dir, portion, imgs_dir_post_fix, f"{i:05d}.png")
            objs_and_bbs = map_to_camera_bbs(
                generate_image(objects, ground_view_width))
            generate_label(os.path.join(dataset_dir, portion), i, objs_and_bbs)

    #   Move scene objects around
    #   Render
    #   Save images and associated labels
    pass


if __name__ == "__main__":
    main()
