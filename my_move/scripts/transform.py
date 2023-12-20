from abc import ABC, abstractmethod

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


class Transformation(ABC):
    def __init__(self, camera_x_offset=0.050, camera_y_offset=-0.06475,
                 camera_z_offset=-0.070864):
        # Initialize the camera offset values
        self.camera_x_offset = camera_x_offset
        self.camera_y_offset = camera_y_offset
        self.camera_z_offset = camera_z_offset

    def marker_to_camera(self, rvecs: np.array = np.eye(3),
                         tvecs: np.array = np.eye(3),
                         point: tuple = (0, 0, 0)):
        """
        Transforms a point from the marker_model coordinate system to the camera
        coordinate system.
        :param rvecs: The rotation vector of the marker_model.
        :param tvecs: The translation vector of the marker_model.
        :param point: The point to be transformed.

        :return: The point transformed and the rotation matrix.
        """
        # Rotation matrix
        r = cv2.Rodrigues(rvecs)[0]

        # Translation matrix
        t = np.array([[tvecs[0]], [tvecs[1]], [tvecs[2]]])

        # Transformation matrix
        transformation_matrix = np.concatenate((r, t), axis=1)
        transformation_matrix = np.concatenate(
            (transformation_matrix, np.array([[0, 0, 0, 1]])), axis=0)

        # Point
        point = np.array([[point[0]], [point[1]], [point[2]], [1]])

        # Point transformation
        point_transformed = transformation_matrix @ point

        return point_transformed, r

    def camera_to_gripper(self, point, safety: bool = True,
                          grasp_z_offset=0.044, medicine_y_offset=0.05):
        """
        Transforms a point from the camera coordinate system to the gripper
        coordinate system.
        :param point: The point to be transformed.
        :param safety: If True, the point will be transformed to the safety position.
        :param grasp_z_offset: The offset in the z axis of the gripper.
        :param medicine_y_offset: The offset in the y axis of the gripper.

        :return: The point transformed and the rotation matrix.
        """
        # Define rotation angles (0 if there is no rotation in relation to the camera axis)
        theta_x = np.radians(0)
        theta_y = np.radians(0)
        theta_z = np.radians(0)

        # Define rotation matrices in axis x, y and z
        rx = np.array([[1, 0, 0],
                       [0, np.cos(theta_x), -np.sin(theta_x)],
                       [0, np.sin(theta_x), np.cos(theta_x)]])

        ry = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                       [0, 1, 0],
                       [-np.sin(theta_y), 0, np.cos(theta_y)]])

        rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                       [np.sin(theta_z), np.cos(theta_z), 0],
                       [0, 0, 1]])

        # Define the rotation matrix
        r = rx @ ry @ rz

        # Define the translation matrix
        safety_z_offset = 0.15

        t = np.array([[self.camera_x_offset],
                      [self.camera_y_offset - medicine_y_offset - 0.02],
                      [self.camera_z_offset - (
                          safety_z_offset if safety else grasp_z_offset)]
                      ])

        # Define the transformation matrix
        transformation_matrix = np.concatenate((r, t), axis=1)
        transformation_matrix = np.concatenate(
            (transformation_matrix, np.array([[0, 0, 0, 1]])), axis=0)

        # Point transformation
        point_transformed = transformation_matrix @ point

        return point_transformed, r

    @abstractmethod
    def robot_transformation_matrix(self,
                                    theta_1: float = 0, theta_2: float = 0,
                                    theta_3: float = 0, theta_4: float = 0,
                                    theta_5: float = 0, theta_6: float = 0):
        """
        Calculates the transformation matrix of the transformation.
        """
        pass

    def gripper_to_base(self, transform_matrix, point: tuple):
        """
        Transforms a point from the gripper coordinate system to the base
        coordinate system.
        :param transform_matrix: The transformation matrix of the transformation.
        :param point: The point to be transformed.

        :return: The point transformed and the rotation matrix.
        """
        point_transformed = transform_matrix @ point

        r = transform_matrix[:3, :3]

        return point_transformed, r

    def marker_to_base(self, rvecs=None, tvecs=None, joint_angles: list = None,
                       y=0,
                       z=0, safety: bool = True):
        """
        Transforms a point from the marker_model coordinate system to the base
        coordinate system.
        :param rvecs: The rotation vector of the marker_model.
        :param tvecs: The translation vector of the marker_model.
        :param joint_angles: The joint angles of the transformation.
        :param y: The offset in the y axis of the gripper.
        :param z: The offset in the z axis of the gripper.
        :param safety: If True, the point will be transformed to the safety position.

        :return: The point transformed and the rotation matrix.
        """
        if rvecs is None:
            rvecs = [0, 0, 0]

        if tvecs is None:
            tvecs = [0, 0, 0]

        if joint_angles is None:
            joint_angles = [0, 0, 0, 0, 0, 0]

        point_a_c, rot_a_c = self.marker_to_camera(rvecs, tvecs)
        point_c_g, rot_c_g = self.camera_to_gripper(point_a_c, safety=safety,
                                                    grasp_z_offset=z,
                                                    medicine_y_offset=y)

        matrix_g_b = self.robot_transformation_matrix(joint_angles[0],
                                                      joint_angles[1],
                                                      joint_angles[2],
                                                      joint_angles[3],
                                                      joint_angles[4],
                                                      joint_angles[5])

        point_a_b, rot_g_b = self.gripper_to_base(matrix_g_b, point_c_g)

        final_point = np.array(
            [point_a_b[0][0], point_a_b[1][0], point_a_b[2][0]])

        rotation_matrix = rot_g_b @ rot_c_g @ rot_a_c

        r = R.from_matrix(rotation_matrix)
        new_gripper_angles = np.degrees(r.as_euler('xyz'))

        # new_gripper_angles[0] = 180 - abs(new_gripper_angles[0])  # [degrees]
        new_gripper_angles[0] = 135 - abs(new_gripper_angles[0])  # [degrees]
        new_gripper_angles[1] = - new_gripper_angles[1]  # [degrees]
        new_gripper_angles[2] = abs(180 + new_gripper_angles[2])  # [degrees]

        final_coordinates = np.concatenate((final_point, new_gripper_angles),
                                           axis=0)

        return final_coordinates