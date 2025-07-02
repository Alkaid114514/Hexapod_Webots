from controller import Robot, InertialUnit
import numpy as np

class IMU:
    def __init__(self, robot: Robot = None):
        """
        初始化IMU传感器
        :param robot: Webots机器人实例
        """
        if robot is not None:
            self.imu = robot.getDevice("imu")
            self.imu.enable(int(robot.getBasicTimeStep()))
        else:
            self.imu = None

    def getRoll(self) -> float:
        """
        获取滚转角 (绕x轴旋转)
        :return: 滚转角(弧度)
        """
        if self.imu is None:
            return 0.0
        return float(self.imu.getRollPitchYaw()[0])

    def getPitch(self) -> float:
        """
        获取俯仰角 (绕y轴旋转)
        :return: 俯仰角(弧度)
        """
        if self.imu is None:
            return 0.0
        return float(self.imu.getRollPitchYaw()[1])

    def getYaw(self) -> float:
        """
        获取偏航角 (绕z轴旋转)
        :return: 偏航角(弧度)
        """
        if self.imu is None:
            return 0.0
        return float(self.imu.getRollPitchYaw()[2])

    def getRollPitchYaw(self) -> np.ndarray:
        """
        获取三轴姿态角 (滚转、俯仰、偏航)
        :return: NumPy数组 [roll, pitch, yaw] (弧度)
        """
        if self.imu is None:
            return np.zeros(3)
        return np.array(self.imu.getRollPitchYaw(), dtype=np.float32)