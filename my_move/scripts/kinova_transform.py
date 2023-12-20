from math import pi

import numpy as np

from transform import Transformation


class KinovaTransformation(Transformation):
    def robot_transformation_matrix(self, theta_1: float = 0,
                                    theta_2: float = 0,
                                    theta_3: float = 0, theta_4: float = 0,
                                    theta_5: float = 0, theta_6: float = 0):
        """
        As specified in the robot's user guide.
        """
        theta_1 = np.radians(theta_1)  # [rad]
        theta_2 = np.radians(theta_2)  # [rad]
        theta_3 = np.radians(theta_3)  # [rad]
        theta_4 = np.radians(theta_4)  # [rad]
        theta_5 = np.radians(theta_5)  # [rad]
        theta_6 = np.radians(theta_6)  # [rad]
        theta_7 = -pi / 2  # [rad]

        m_b_1 = np.array([[np.cos(theta_1), -np.sin(theta_1), 0, 0],
                          [np.sin(theta_1), np.cos(theta_1), 0, 0],
                          [0, 0, 1, 0.1283],
                          [0, 0, 0, 1]])

        m_1_2 = np.array([[np.cos(theta_2), -np.sin(theta_2), 0, 0],
                          [0, 0, -1, -0.03],
                          [np.sin(theta_2), np.cos(theta_2), 0, 0.115],
                          [0, 0, 0, 1]])

        m_2_3 = np.array([[np.cos(theta_3), -np.sin(theta_3), 0, 0],
                          [-np.sin(theta_3), -np.cos(theta_3), 0, 0.28],
                          [0, 0, -1, 0],
                          [0, 0, 0, 1]])

        m_3_4 = np.array([[np.cos(theta_4), -np.sin(theta_4), 0, 0],
                          [0, 0, -1, -0.14],
                          [np.sin(theta_4), np.cos(theta_4), 0, 0.02],
                          [0, 0, 0, 1]])

        m_4_5 = np.array([[0, 0, 1, 0.0285],
                          [np.sin(theta_5), np.cos(theta_5), 0, 0],
                          [-np.cos(theta_5), np.sin(theta_5), 0, 0.105],
                          [0, 0, 0, 1]])

        m_5_6 = np.array([[0, 0, -1, -0.105],
                          [np.sin(theta_6), np.cos(theta_6), 0, 0],
                          [np.cos(theta_6), -np.sin(theta_6), 0, 0.0285],
                          [0, 0, 0, 1]])

        m_6_ee = np.array([[np.cos(theta_7), -np.sin(theta_7), 0, 0],
                           [np.sin(theta_7), np.cos(theta_7), 0, 0],
                           [0, 0, 1, 0.13],
                           [0, 0, 0, 1]])

        transformation_matrix = m_b_1 @ m_1_2 @ m_2_3 @ m_3_4 @ m_4_5 @ m_5_6 @ m_6_ee

        return transformation_matrix