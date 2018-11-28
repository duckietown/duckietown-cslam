import contracts
import duckietown_world as dw
import geometry as g
import numpy as np

from duckietown_world.svg_drawing.ipython_utils import ipython_draw_svg

watchtower_height = 0.6
map_ = dw.load_map('robotarium1')


def get_all_floor_april_tags(map_):
    """
        Returns all April tags on the floor in the given map.

        Input: - map_: DuckietownMap storing the information for the map.

        Output: List of tuples (name, pose) of the floor April tags found in th
                map. name is the name associated to the tag (e.g. the 'cell
                position'), whereas pose is the pose of the (center of the) tag
                w.r.t. to the world frame.

    """
    april_tags = []
    for element in map_.spatial_relations.values():
        # Check if element being considered is a FloorTag
        name = element.b[0]
        child = map_.children[name]
        if type(child) == dw.world_duckietown.tags_db.FloorTag:
            # Get transform
            p = element.transform.p
            theta = element.transform.theta
            transform = g.SE3_from_SE2(g.SE2_from_translation_angle(p, theta))
            april_tags.append((name, transform))

    return april_tags


def get_all_street_sign_april_tags(map_):
    """
        Returns all April tags on the street signs (if they start for 'tag').

        Input: - map_: DuckietownMap storing the information for the map.

        Output: List of tuples (name, pose) of the floor April tags found in th
                map. name is the name associated to the tag (e.g. the 'cell
                position'), whereas pose is the pose of the (center of the) tag
                w.r.t. to the world frame.

    """
    april_tags = []
    for element in map_.spatial_relations.values():
        # Check if the name of the element starts for 'tag'
        name = element.b[0]
        if name.find('tag', 0) != -1:
            # Get transform
            p = element.transform.p
            theta = element.transform.theta
            transform = g.SE3_from_SE2(g.SE2_from_translation_angle(p, theta))
            april_tags.append((name, transform))

    return april_tags


def get_tag_coordinates_in_camera_frame(tag, camera_pose_wrt_world):
    """
        Returns the coordinates of the center of the given tag in the camera
        frame, if the tag is visible by the camera. The latter has pose
        camera_pose_wrt_world in the world coordinate frame.

        Input: - tag: April tag in the form of a tuple (name, tag_pose), with
                      name being the name associated to the tag (e.g. 'O7', its
                      'cell position' in the robotarium) and tag_pose being an
                      SE3 4x4 matrix representing the pose of the tag in the
                      world frame.
               - camera_pose_wrt_world: SE3 4x4 matrix representing the pose of
                                        the camera in the world frame.

        Output: - If the tag is visible by the camera: an SE3 4x4 matrix
                  representing the pose of the tag in the camera frame. The
                  position is referred to the center of the tag.
                - If the tag is not visible by the camera: an empty numpy array.
    """
    # Intrinsic matrix (made up on the basis of the results of a calibration of
    # a real Duckiebot camera).
    image_size = (480, 640)
    K = np.array([[350., 0., 330., 0.], [0., 350., 255., 0.], [0., 0., 1., 0.]])
    # Extrinsic matrix (note that the input pose is camera w.r.t. world)
    E = g.SE3.inverse(camera_pose_wrt_world)
    # Check if tag is visible from camera
    tag_center_wrt_world_homo = tag[1][:, 3]
    tag_center_pixel_coord_homo = np.matmul(
        np.matmul(K, E), tag_center_wrt_world_homo)
    if tag_center_pixel_coord_homo[-1] != 1.0:
        tag_center_pixel_coord_homo /= tag_center_pixel_coord_homo[-1]
    if 0 <= tag_center_pixel_coord_homo[0] <= image_size[0] and \
       0 <= tag_center_pixel_coord_homo[1] <= image_size[1]:
        # Seen by camera -> Return pose w.r.t camera
        print("- Tag {0} seen, with coordinates ({1}, {2})".format(
              tag[0], tag_center_pixel_coord_homo[0],
            tag_center_pixel_coord_homo[1]))
        return g.SE3.multiply(E, tag[1])
    else:
        return np.array([])


def extract_poses():
    # Draw map
    #ipython_draw_svg(map_)

    # Obtain FloorTags with poses
    tags = get_all_floor_april_tags(map_)

    tags_seen = {}

    #transform = map_.spatial_relations[3].transform
    #R_2D = transform.as_SE2()[:2,:2]
    #p = transform.p
    #R = g.SO3_from_SO2(R_2D)
    #T = np.hstack([p, watchtower_height])
    #camera_pose = g.SE3_from_rotation_translation(R,T)

    # Test camera poses: put watchtowers where the street signs are
    for name, pose in get_all_street_sign_april_tags(map_):
        R = pose[:3, :3]
        p = pose[:2, 3]
        T = np.hstack([p, watchtower_height])
        camera_pose = g.SE3_from_rotation_translation(R, T)
        camera_index = len(tags_seen.keys())

        print("*** The camera placed where the {} street sign is sees the "
              "following:".format(name))
        tags_seen[camera_index] = []
        for tag in tags:
            coord = get_tag_coordinates_in_camera_frame(tag, camera_pose)
            if coord.size > 0:
                # Non-empty
                tags_seen[camera_index].append((tag, name))

    return tags, tags_seen


if __name__ == '__main__':
    tags, tags_seen = extract_poses()
