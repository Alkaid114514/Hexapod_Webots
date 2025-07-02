import math
import numpy as np
from controller import Motor, TouchSensor
from hex_leg_interface import ILeg

COXA_LEN = 38.0 / 1000.0
FEMUR_LEN = 79.2 / 1000.0
TIBIA_LEN = 116.0 / 1000.0

def CLIP(value, lower, upper):
    return lower if value < lower else upper if value > upper else value

class Leg(ILeg):
    def __init__(self):
        self.coxaMotor = None
        self.femurMotor = None
        self.tibiaMotor = None
        self.motorAngles = np.zeros(3)
        self.isOnGround = True
        self.initAngles = np.array([0.0, 0.0, -math.pi / 3.0])
        self.currentStandAngles = self.initAngles.copy()
        self.ctr2root = np.zeros(3)
        self.ctr2rootTheta = 0.0
        self.currentYaw = 0.0
        self.currentRoll = 0.0
        self.currentPitch = 0.0
        self.currentPosition = np.zeros(3)
        self.initStandBodyTarget = np.zeros(3)
        self.currentStandBodyTarget = np.zeros(3)
        self.lastBodyTarget = np.zeros(3)
        self.touchSensor = None

    def __init__(self, coxa: Motor, femur: Motor, tibia: Motor, ctr2root: np.ndarray, ctr2rootTheta: float):
        self.coxaMotor = coxa
        self.femurMotor = femur
        self.tibiaMotor = tibia
        self.ctr2root = ctr2root.copy()
        self.ctr2rootTheta = ctr2rootTheta
        self.motorAngles = np.zeros(3)
        self.isOnGround = True
        self.initAngles = np.array([0.0, 0.0, -math.pi / 3.0])
        self.currentStandAngles = self.initAngles.copy()
        self.currentYaw = 0.0
        self.currentRoll = 0.0
        self.currentPitch = 0.0
        self.currentPosition = np.zeros(3)
        self.initStandBodyTarget = np.zeros(3)
        self.currentStandBodyTarget = np.zeros(3)
        self.lastBodyTarget = np.zeros(3)
        self.touchSensor = None

    def initialize(self):
        self.initStandBodyTarget = self.leg2bodyCoord(self.fk(self.initAngles), self.ctr2root, self.ctr2rootTheta)
        self.currentStandBodyTarget = self.initStandBodyTarget.copy()
        self.lastBodyTarget = self.initStandBodyTarget.copy()
    @staticmethod
    def leg2bodyCoord(relevant, bias, theta):
        relevant = relevant.copy()
        relevant[2] = -relevant[2]
        relevant[1] = -relevant[1]
        rotation_matrix = np.array([
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ])
        v = np.dot(rotation_matrix, relevant)
        return v + bias

    def setMotor(self, coxa: Motor, femur: Motor, tibia: Motor):
        self.coxaMotor = coxa
        self.femurMotor = femur
        self.tibiaMotor = tibia

    def setOmega(self, omega: np.ndarray):
        self.coxaMotor.setVelocity(omega[0])
        self.femurMotor.setVelocity(omega[1])
        self.tibiaMotor.setVelocity(omega[2])

    def setCoxaOmega(self, omega: float):
        self.coxaMotor.setVelocity(omega)

    def setFemurOmega(self, omega: float):
        self.femurMotor.setVelocity(omega)

    def setTibiaOmega(self, omega: float):
        self.tibiaMotor.setVelocity(omega)

    def setLegTarget(self, target: np.ndarray):
        self.setPose(self.ik(target))

    def setBodyTarget(self, target: np.ndarray):
        self.lastBodyTarget = target.copy()
        leg_target = self.body2legCoord(target, self.ctr2root, self.ctr2rootTheta)
        self.setLegTarget(leg_target)

    def setCoxaPose(self, angle: float):
        self.motorAngles[0] = angle

    def setFemurPose(self, angle: float):
        self.motorAngles[1] = angle

    def reInit(self):
        self.setPose(self.currentStandAngles)

    def setHeight(self, height: float):
        self.currentStandBodyTarget[2] = -height
        self.lastBodyTarget = self.currentStandBodyTarget.copy()
        leg_coords = self.body2legCoord(self.currentStandBodyTarget, self.ctr2root, self.ctr2rootTheta)
        self.currentStandAngles = self.ik(leg_coords)

    def setYaw(self, yaw: float):
        rotation_matrix = np.array([
            [math.cos(yaw - self.currentYaw), -math.sin(yaw - self.currentYaw), 0],
            [math.sin(yaw - self.currentYaw), math.cos(yaw - self.currentYaw), 0],
            [0, 0, 1]
        ])
        self.ctr2root = np.dot(rotation_matrix, self.ctr2root)
        self.currentYaw = yaw
        leg_coords = self.body2legCoord(self.currentStandBodyTarget, self.ctr2root, self.ctr2rootTheta)
        self.currentStandAngles = self.ik(leg_coords)

    def setRoll(self, roll: float):
        rotation_matrix = np.array([
            [math.cos(roll - self.currentRoll), 0, -math.sin(roll - self.currentRoll)],
            [0, 1, 0],
            [math.sin(roll - self.currentRoll), 0, math.cos(roll - self.currentRoll)]
        ])
        self.currentStandBodyTarget = np.dot(rotation_matrix, self.currentStandBodyTarget)
        self.currentRoll = roll
        leg_coords = self.body2legCoord(self.currentStandBodyTarget, self.ctr2root, self.ctr2rootTheta)
        self.currentStandAngles = self.ik(leg_coords)

    def setPitch(self, pitch: float):
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, math.cos(pitch - self.currentPitch), math.sin(pitch - self.currentPitch)],
            [0, -math.sin(pitch - self.currentPitch), math.cos(pitch - self.currentPitch)]
        ])
        self.currentStandBodyTarget = np.dot(rotation_matrix, self.currentStandBodyTarget)
        self.currentPitch = pitch
        leg_coords = self.body2legCoord(self.currentStandBodyTarget, self.ctr2root, self.ctr2rootTheta)
        self.currentStandAngles = self.ik(leg_coords)
    @staticmethod
    def body2legCoord(absolute, bias, theta):
        v = absolute - bias
        rotation_matrix = np.array([
            [math.cos(theta), math.sin(theta), 0],
            [-math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ])
        v = np.dot(rotation_matrix, v)
        v[1] = -v[1]
        v[2] = -v[2]
        return v

    def setBodyPosition(self, bodyPos: np.ndarray):
        bodyPos[2] = 0.0
        self.ctr2root -= (bodyPos - self.currentPosition)
        self.currentPosition = bodyPos.copy()
        leg_coords = self.body2legCoord(self.currentStandBodyTarget, self.ctr2root, self.ctr2rootTheta)
        self.currentStandAngles = self.ik(leg_coords)

    def toGround(self, height: float):
        self.isOnGround = bool(int(self.touchSensor.getValue())) or (-self.lastBodyTarget[2] >= 1.5 * height)
        if not self.isOnGround:
            self.lastBodyTarget[2] -= 0.003
            self.setBodyTarget(self.lastBodyTarget)

    def checkIsOnGroud(self, height: float):
        self.isOnGround = bool(int(self.touchSensor.getValue())) or (-self.lastBodyTarget[2] >= 1.5 * height)

    def moveToGround(self, currentHeight: float):
        pass

    def startMotor(self):
        self.coxaMotor.setPosition(self.motorAngles[0])
        self.femurMotor.setPosition(self.motorAngles[1])
        self.tibiaMotor.setPosition(self.motorAngles[2])
        # self.isOnGround = bool(int(self.touchSensor.getValue()))


class LegL(Leg):
    def fk(self, angles: np.ndarray) -> np.ndarray:
        tmp = COXA_LEN + FEMUR_LEN * math.cos(angles[1]) + TIBIA_LEN * math.cos(-angles[2] - angles[1])
        v = np.array([
            tmp * math.cos(angles[0]),
            tmp * math.sin(angles[0]),
            -FEMUR_LEN * math.sin(angles[1]) + TIBIA_LEN * math.sin(-angles[2] - angles[1])
        ])
        return v

    def ik(self, vector3: np.ndarray) -> np.ndarray:
        vector3 = vector3.copy()
        vector3[2] = -vector3[2]
        theta1 = math.atan2(vector3[1], vector3[0])
        R = math.sqrt(vector3[0]**2 + vector3[1]**2)
        ar = math.atan2(-vector3[2], (R - COXA_LEN))
        Lr = math.sqrt(vector3[2]**2 + (R - COXA_LEN)**2)
        a1 = math.acos(CLIP((FEMUR_LEN**2 + Lr**2 - TIBIA_LEN**2) / (2 * Lr * FEMUR_LEN), -1, 1))
        theta2 = a1 - ar
        a2 = math.acos(CLIP((Lr**2 + TIBIA_LEN**2 - FEMUR_LEN**2) / (2 * Lr * TIBIA_LEN), -1, 1))
        theta3 = -(a1 + a2)
        return np.array([theta1, -theta2, theta3])

    def setPose(self, angles: np.ndarray):
        angles = angles.copy()
        angles[2] += math.pi / 3.0
        self.motorAngles = angles

    def setTibiaPose(self, angle: float):
        angle += math.pi / 3.0
        self.motorAngles[2] = angle


class LegR(Leg):
    def fk(self, angles: np.ndarray) -> np.ndarray:
        tmp = COXA_LEN + FEMUR_LEN * math.cos(angles[1]) + TIBIA_LEN * math.cos(angles[2] - angles[1])
        v = np.array([
            tmp * math.cos(angles[0]),
            tmp * math.sin(angles[0]),
            -FEMUR_LEN * math.sin(angles[1]) + TIBIA_LEN * math.sin(angles[2] - angles[1])
        ])
        return v

    def ik(self, vector3: np.ndarray) -> np.ndarray:
        vector3 = vector3.copy()
        vector3[2] = -vector3[2]
        theta1 = math.atan2(vector3[1], vector3[0])
        R = math.sqrt(vector3[0]**2 + vector3[1]**2)
        ar = math.atan2(-vector3[2], (R - COXA_LEN))
        Lr = math.sqrt(vector3[2]**2 + (R - COXA_LEN)**2)
        a1 = math.acos(CLIP((FEMUR_LEN**2 + Lr**2 - TIBIA_LEN**2) / (2 * Lr * FEMUR_LEN), -1, 1))
        theta2 = a1 - ar
        a2 = math.acos(CLIP((Lr**2 + TIBIA_LEN**2 - FEMUR_LEN**2) / (2 * Lr * TIBIA_LEN), -1, 1))
        theta3 = -(a1 + a2)
        return np.array([theta1, theta2, -theta3])

    def setPose(self, angles: np.ndarray):
        angles = angles.copy()
        angles[2] -= math.pi / 3.0
        self.motorAngles = angles

    def setTibiaPose(self, angle: float):
        angle -= math.pi / 3.0
        self.motorAngles[2] = angle