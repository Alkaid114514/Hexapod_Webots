import math
import numpy as np
from controller import Robot,Display
from HexLeg import LegR
from HexLeg import LegL
from HexIMU import IMU
from HexCamera import HexCamera
from HexLaserLidar import LaserLidar

COXA_LEN = 38.0 / 1000.0
FEMUR_LEN = 79.2 / 1000.0
TIBIA_LEN = 116.0 / 1000.0

WAVE_RATIO = 1.0 / 5.0
RIPPLE_RATIO = 1.0 / 2.0
FOURPLUSTWO_RATIO = 1.0 / 3.0

def CLIP(value, lower, upper):
    return lower if value < lower else upper if value > upper else value

class GaitStatus:
    Tripod = 0
    Ripple = 1
    Wave = 2
    Ready = 3
    Stop = 4
    FourPlusTwo = 5

class Hexapod(Robot):
    def __init__(self):
        super().__init__()
        
        self.imu = IMU(self)
        # self.camera = HexCamera(self)
        self.lidar = None
        self.timeStep = int(self.getBasicTimeStep())

        # # 获取内置显示器
        # self.display = self.getDevice("display")
        # if self.display:
        #     print("内置显示器已启用")
        # else:
        #     print("警告: 找不到内置显示设备")

        # # 初始化相机相关参数
        # self.display_counter = 0
        # self.display_interval = 5  # 每5个时间步更新一次显示

        # Initialize legs with numpy arrays
        self.BRleg = LegR(self.getDevice("M_BR_COXA"), self.getDevice("M_BR_FEMUR"), self.getDevice("M_BR_TIBIA"), 
                         self.ctr2BRroot, self.ctr2BRrootTheta)
        self.MRleg = LegR(self.getDevice("M_MR_COXA"), self.getDevice("M_MR_FEMUR"), self.getDevice("M_MR_TIBIA"), 
                         self.ctr2MRroot, self.ctr2MRrootTheta)
        self.FRleg = LegR(self.getDevice("M_FR_COXA"), self.getDevice("M_FR_FEMUR"), self.getDevice("M_FR_TIBIA"), 
                         self.ctr2FRroot, self.ctr2FRrootTheta)
        self.BLleg = LegL(self.getDevice("M_BL_COXA"), self.getDevice("M_BL_FEMUR"), self.getDevice("M_BL_TIBIA"), 
                         self.ctr2BLroot, self.ctr2BLrootTheta)
        self.MLleg = LegL(self.getDevice("M_ML_COXA"), self.getDevice("M_ML_FEMUR"), self.getDevice("M_ML_TIBIA"), 
                         self.ctr2MLroot, self.ctr2MLrootTheta)
        self.FLleg = LegL(self.getDevice("M_FL_COXA"), self.getDevice("M_FL_FEMUR"), self.getDevice("M_FL_TIBIA"), 
                         self.ctr2FLroot, self.ctr2FLrootTheta)
        
        # Initialize legs
        self.BRleg.initialize()
        self.MRleg.initialize()
        self.FRleg.initialize()
        self.BLleg.initialize()
        self.MLleg.initialize()
        self.FLleg.initialize()
        
        # Setup touch sensors
        self.BLleg.touchSensor = self.getDevice("BL")
        self.MLleg.touchSensor = self.getDevice("ML")
        self.FLleg.touchSensor = self.getDevice("FL")
        self.BRleg.touchSensor = self.getDevice("BR")
        self.MRleg.touchSensor = self.getDevice("MR")
        self.FRleg.touchSensor = self.getDevice("FR")
        
        # Enable touch sensors
        self.BLleg.touchSensor.enable(self.timeStep)
        self.MLleg.touchSensor.enable(self.timeStep)
        self.FLleg.touchSensor.enable(self.timeStep)
        self.BRleg.touchSensor.enable(self.timeStep)
        self.MRleg.touchSensor.enable(self.timeStep)
        self.FRleg.touchSensor.enable(self.timeStep)
        
        # Calculate initial height using numpy array
        leg_coords = self.body2legCoord(
                self.FLleg.initStandBodyTarget, self.FLleg.ctr2root, self.FLleg.ctr2rootTheta)
        self.currentHeight = self.initHeight = leg_coords[2]

    # def update_display(self):
    #     """在Webots内置显示器上显示相机画面"""
    #     # 如果没有显示器或没有图像数据，直接返回
    #     if not self.display or not self.camera or not self.camera.image_data:
    #         return
        
    #     # 获取图像数据
    #     image_data = self.camera.image_data
    #     width = self.camera.width
    #     height = self.camera.height
        
        # 创建Webots图像对象
        # image_ref = self.display.imageNew(
        #     data=image_data,
        #     width=width,
        #     height=height,
        #     width = width * 4,//这里老是击败报错补吱道什么原因，说参数数量不对
        #     format=self.display.BGRA
        # )
        
        # # 显示图像
        # self.display.imagePaste(image_ref, 0, 0, False)
        
        # # 删除图像引用以释放内存
        # self.display.imageDelete(image_ref)
    
    # def step(self, timestep):
    #     """重载step方法以包含相机更新"""
    #     result = super().step(timestep)
        
    #     # 更新相机显示
    #     if self.camera:
    #         self.camera.update()
            
    #         # 更新计数器
    #         self.display_counter += 1
    #         if self.display_counter >= self.display_interval:
    #             self.update_display()
    #             self.display_counter = 0
        
    #     return result
    
    def init_lidar(self, device_name="lidar_sensor"):
        """激光雷达初始化"""
        self.lidar = LaserLidar(device_name)
        self.lidar.enable()
        print(f"LiDAR enabled with {self.lidar.get_fov()['horizontal']*180/3.14:.1f}° FOV")
    
    def collect_sensor_data(self):
        """传感器数据收集入口"""
        sensor_data = {}
        if self.lidar:
            sensor_data['lidar'] = {
                'points': self.lidar.get_point_cloud(),
                'range_image': self.lidar.get_range_image()
            }
        return sensor_data
    
    # Constants for leg positions as numpy arrays
    ctr2BRroot = np.array([0.059, -0.083, 0.0])
    ctr2MRroot = np.array([0.08, 0.0, 0.0])
    ctr2FRroot = np.array([0.064, 0.082, 0.0])
    ctr2BLroot = np.array([-0.059, -0.083, 0.0])
    ctr2MLroot = np.array([-0.08, 0.0, 0.0])
    ctr2FLroot = np.array([-0.064, 0.082, 0.0])
    
    # Constants for leg angles
    ctr2BRrootTheta = -math.pi / 4
    ctr2MRrootTheta = 0.0
    ctr2FRrootTheta = math.pi / 4
    ctr2BLrootTheta = -3 * math.pi / 4
    ctr2MLrootTheta = math.pi
    ctr2FLrootTheta = 3 * math.pi / 4
    
    # Other member variables
    initHeight = 0.0
    currentHeight = 0.0
    coxaOmega = 0.1
    femurOmega = 0.1
    tibiaOmega = 0.1
    totalFrame = 32
    stepTheta = 0.0
    stepLen = 0.05
    
    gaitGroupIndex = 0
    previousGroupIndex = -1
    frame = totalFrame // 2
    velocity = np.array([0.0, 0.0, 0.0])
    omega = 0.0
    lockedVelocity = np.array([0.0, 0.0, 0.0])
    lockedOmega = 0.0
    gaitStatus = GaitStatus.Stop
    lockedR = np.array([0.0, 0.0, 0.0])
    
    currentPitch = 0.0
    currentRoll = 0.0
    isBalanced = True
    
    def setPose(self, BRangles, MRangles, FRangles, BLangles, MLangles, FLangles):
        self.BRleg.setPose(BRangles)
        self.MRleg.setPose(MRangles)
        self.FRleg.setPose(FRangles)
        self.BLleg.setPose(BLangles)
        self.MLleg.setPose(MLangles)
        self.FLleg.setPose(FLangles)
    
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
    
    def prepareNextCycle(self, moveStatus):
        if self.isGaitCycleStart() or self.gaitStatus == GaitStatus.Stop:
            if np.array_equal(self.velocity, np.zeros(3)) and self.omega == 0.0:
                self.reInit()
                self.startMove()
                self.gaitStatus = GaitStatus.Stop
                return False
            
            self.lockedOmega = self.omega if self.omega != 0.0 else np.finfo(float).eps
            w = np.array([0.0, 0.0, self.lockedOmega])
            self.lockedVelocity = self.velocity.copy()
            isVzero = np.array_equal(self.lockedVelocity, np.zeros(3))
            
            vel_magnitude_sq = np.sum(self.lockedVelocity**2)
            self.totalFrame = 24 if isVzero else int(math.ceil(32 * (0.01 / vel_magnitude_sq)))
            
            mr_target = self.MRleg.currentStandBodyTarget
            if isVzero:
                self.stepLen = self.lockedOmega * math.sqrt(mr_target[0]**2 + mr_target[1]**2)
            else:
                self.stepLen = (self.lockedOmega / abs(self.lockedOmega)) * np.linalg.norm(self.lockedVelocity)
            
            self.stepLen *= (self.timeStep / 1000.0) * self.totalFrame
            
            self.lockedR = np.cross(self.lockedVelocity, w) / (self.lockedOmega**2)
            rlen = math.sqrt(mr_target[0]**2 + mr_target[1]**2) if isVzero else np.linalg.norm(self.lockedR)
            self.stepTheta = self.stepLen / rlen
            self.gaitStatus = moveStatus
        
        return True
    
    def getSwagNextBodyTarget(self, r0, currentStandBodyTarget):
        pc = r0 + currentStandBodyTarget
        theta = self.stepTheta * self.frame / self.totalFrame - self.stepTheta / 2.0
        rotation_matrix = np.array([
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ])
        pt = np.dot(rotation_matrix, pc)
        t = pt - r0
        t[2] += 0.06 * math.sin((self.frame / self.totalFrame) * math.pi)
        return t
    
    def getStandNextBodyTarget(self, r0, currentStandBodyTarget, baseRatio=1.0, ratio=0.0):
        pc = r0 + currentStandBodyTarget
        theta = self.stepTheta * ratio + self.stepTheta * baseRatio * self.frame / self.totalFrame - self.stepTheta / 2.0
        rotation_matrix = np.array([
            [math.cos(theta), math.sin(theta), 0],
            [-math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ])
        pt = np.dot(rotation_matrix, pc)
        t = pt - r0
        return t

    # ... (rest of the methods remain largely the same, just using numpy arrays instead of Vector3)
    # For example, the moveRipple, move4plus2, moveWave, moveTripod methods would use numpy arrays
    
    def setBodyTargets(self, BRtarget, MRtarget, FRtarget, BLtarget, MLtarget, FLtarget):
        self.BRleg.setLegTarget(self.body2legCoord(BRtarget, self.BRleg.ctr2root, self.BRleg.ctr2rootTheta))
        self.MRleg.setLegTarget(self.body2legCoord(MRtarget, self.MRleg.ctr2root, self.MRleg.ctr2rootTheta))
        self.FRleg.setLegTarget(self.body2legCoord(FRtarget, self.FRleg.ctr2root, self.FRleg.ctr2rootTheta))
        self.BLleg.setLegTarget(self.body2legCoord(BLtarget, self.BLleg.ctr2root, self.BLleg.ctr2rootTheta))
        self.MLleg.setLegTarget(self.body2legCoord(MLtarget, self.MLleg.ctr2root, self.MLleg.ctr2rootTheta))
        self.FLleg.setLegTarget(self.body2legCoord(FLtarget, self.FLleg.ctr2root, self.FLleg.ctr2rootTheta))
    
    # ... (other methods like setHeight, setYaw, etc. remain the same)
    def checkIsOnGround(self):
        pass  # Implement if needed
    
    def moveRipple(self):
        if not self.prepareNextCycle(GaitStatus.Ripple):
            return
        
        if self.gaitGroupIndex == 0:
            self.FLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.FLleg.currentStandBodyTarget))
            self.MRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.MRleg.currentStandBodyTarget))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, RIPPLE_RATIO, 1.0 * RIPPLE_RATIO))
            self.MLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MLleg.currentStandBodyTarget, RIPPLE_RATIO, 2.0 * RIPPLE_RATIO))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, RIPPLE_RATIO, 2.0 * RIPPLE_RATIO))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, RIPPLE_RATIO, 1.0 * RIPPLE_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.previousGroupIndex = self.gaitGroupIndex
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 1:
            self.MLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.MLleg.currentStandBodyTarget))
            self.BRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.BRleg.currentStandBodyTarget))
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, RIPPLE_RATIO, 1.0 * RIPPLE_RATIO))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, RIPPLE_RATIO, 2.0 * RIPPLE_RATIO))
            self.MRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MRleg.currentStandBodyTarget, RIPPLE_RATIO, 1.0 * RIPPLE_RATIO))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, RIPPLE_RATIO, 2.0 * RIPPLE_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.previousGroupIndex = self.gaitGroupIndex
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 2:
            self.BLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.BLleg.currentStandBodyTarget))
            self.FRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.FRleg.currentStandBodyTarget))
            self.MLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MLleg.currentStandBodyTarget, RIPPLE_RATIO, 1.0 * RIPPLE_RATIO))
            self.MRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MRleg.currentStandBodyTarget, RIPPLE_RATIO, 2.0 * RIPPLE_RATIO))
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, RIPPLE_RATIO, 2.0 * RIPPLE_RATIO))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, RIPPLE_RATIO, 1.0 * RIPPLE_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.previousGroupIndex = self.gaitGroupIndex
                self.gaitGroupIndex -= 2
        
        self.startMove()
        self.frame += 1
    
    def move4plus2(self):
        if not self.prepareNextCycle(GaitStatus.FourPlusTwo):
            return
        
        if self.gaitGroupIndex == 0:
            self.FLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.FLleg.currentStandBodyTarget))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 3.0 * FOURPLUSTWO_RATIO))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 2.0 * FOURPLUSTWO_RATIO))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 1.0 * FOURPLUSTWO_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 1:
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 1.0 * FOURPLUSTWO_RATIO))
            self.FRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.FRleg.currentStandBodyTarget))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 3.0 * FOURPLUSTWO_RATIO))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 2.0 * FOURPLUSTWO_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 2:
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 2.0 * FOURPLUSTWO_RATIO))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 1.0 * FOURPLUSTWO_RATIO))
            self.BLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.BLleg.currentStandBodyTarget))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 3.0 * FOURPLUSTWO_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 3:
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 3.0 * FOURPLUSTWO_RATIO))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 2.0 * FOURPLUSTWO_RATIO))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, FOURPLUSTWO_RATIO, 1.0 * FOURPLUSTWO_RATIO))
            self.BRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.BRleg.currentStandBodyTarget))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex -= 3
        
        self.startMove()
        self.frame += 1
    
    def moveWave(self):
        if not self.prepareNextCycle(GaitStatus.Wave):
            return
        
        if self.gaitGroupIndex == 0:
            self.FLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.FLleg.currentStandBodyTarget))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, WAVE_RATIO, 5.0 * WAVE_RATIO))
            self.MLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MLleg.currentStandBodyTarget, WAVE_RATIO, 4.0 * WAVE_RATIO))
            self.MRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MRleg.currentStandBodyTarget, WAVE_RATIO, 3.0 * WAVE_RATIO))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, WAVE_RATIO, 2.0 * WAVE_RATIO))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, WAVE_RATIO, 1.0 * WAVE_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 1:
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, WAVE_RATIO, 1.0 * WAVE_RATIO))
            self.FRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.FRleg.currentStandBodyTarget))
            self.MLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MLleg.currentStandBodyTarget, WAVE_RATIO, 5.0 * WAVE_RATIO))
            self.MRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MRleg.currentStandBodyTarget, WAVE_RATIO, 4.0 * WAVE_RATIO))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, WAVE_RATIO, 3.0 * WAVE_RATIO))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, WAVE_RATIO, 2.0 * WAVE_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 2:
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, WAVE_RATIO, 2.0 * WAVE_RATIO))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, WAVE_RATIO, 1.0 * WAVE_RATIO))
            self.MLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.MLleg.currentStandBodyTarget))
            self.MRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MRleg.currentStandBodyTarget, WAVE_RATIO, 5.0 * WAVE_RATIO))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, WAVE_RATIO, 4.0 * WAVE_RATIO))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, WAVE_RATIO, 3.0 * WAVE_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 3:
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, WAVE_RATIO, 3.0 * WAVE_RATIO))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, WAVE_RATIO, 2.0 * WAVE_RATIO))
            self.MLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MLleg.currentStandBodyTarget, WAVE_RATIO, 1.0 * WAVE_RATIO))
            self.MRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.MRleg.currentStandBodyTarget))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, WAVE_RATIO, 5.0 * WAVE_RATIO))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, WAVE_RATIO, 4.0 * WAVE_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 4:
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, WAVE_RATIO, 4.0 * WAVE_RATIO))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, WAVE_RATIO, 3.0 * WAVE_RATIO))
            self.MLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MLleg.currentStandBodyTarget, WAVE_RATIO, 2.0 * WAVE_RATIO))
            self.MRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MRleg.currentStandBodyTarget, WAVE_RATIO, 1.0 * WAVE_RATIO))
            self.BLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.BLleg.currentStandBodyTarget))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BRleg.currentStandBodyTarget, WAVE_RATIO, 5.0 * WAVE_RATIO))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 5:
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FLleg.currentStandBodyTarget, WAVE_RATIO, 5.0 * WAVE_RATIO))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.FRleg.currentStandBodyTarget, WAVE_RATIO, 4.0 * WAVE_RATIO))
            self.MLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MLleg.currentStandBodyTarget, WAVE_RATIO, 3.0 * WAVE_RATIO))
            self.MRleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.MRleg.currentStandBodyTarget, WAVE_RATIO, 2.0 * WAVE_RATIO))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(
                self.lockedR, self.BLleg.currentStandBodyTarget, WAVE_RATIO, 1.0 * WAVE_RATIO))
            self.BRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.BRleg.currentStandBodyTarget))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.gaitGroupIndex -= 5
        
        self.startMove()
        self.frame += 1
    
    def moveTripod(self):
        if not self.prepareNextCycle(GaitStatus.Tripod):
            return
        
        if self.gaitGroupIndex == 0:
            self.MLleg.setBodyTarget(self.getStandNextBodyTarget(self.lockedR, self.MLleg.currentStandBodyTarget))
            self.FRleg.setBodyTarget(self.getStandNextBodyTarget(self.lockedR, self.FRleg.currentStandBodyTarget))
            self.BRleg.setBodyTarget(self.getStandNextBodyTarget(self.lockedR, self.BRleg.currentStandBodyTarget))
            
            self.MRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.MRleg.currentStandBodyTarget))
            self.FLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.FLleg.currentStandBodyTarget))
            self.BLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.BLleg.currentStandBodyTarget))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.previousGroupIndex = self.gaitGroupIndex
                self.gaitGroupIndex += 1
        
        elif self.gaitGroupIndex == 1:
            self.MRleg.setBodyTarget(self.getStandNextBodyTarget(self.lockedR, self.MRleg.currentStandBodyTarget))
            self.FLleg.setBodyTarget(self.getStandNextBodyTarget(self.lockedR, self.FLleg.currentStandBodyTarget))
            self.BLleg.setBodyTarget(self.getStandNextBodyTarget(self.lockedR, self.BLleg.currentStandBodyTarget))
            
            self.MLleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.MLleg.currentStandBodyTarget))
            self.FRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.FRleg.currentStandBodyTarget))
            self.BRleg.setBodyTarget(self.getSwagNextBodyTarget(self.lockedR, self.BRleg.currentStandBodyTarget))
            
            if self.isGaitCycleFinish():
                self.frame = -1
                self.previousGroupIndex = self.gaitGroupIndex
                self.gaitGroupIndex -= 1
        
        self.startMove()
        self.frame += 1
    
    def isGaitCycleFinish(self):
        return self.frame > self.totalFrame - 1
    
    def isGaitCycleStart(self):
        return self.frame < 1

    def reInit(self):
        self.BRleg.reInit()
        self.MRleg.reInit()
        self.FRleg.reInit()
        self.BLleg.reInit()
        self.MLleg.reInit()
        self.FLleg.reInit()
    
    def setHeight(self, height):
        self.currentHeight = height
        self.BRleg.setHeight(height)
        self.MRleg.setHeight(height)
        self.FRleg.setHeight(height)
        self.BLleg.setHeight(height)
        self.MLleg.setHeight(height)
        self.FLleg.setHeight(height)
    
    def setYaw(self, yaw):
        self.BRleg.setYaw(yaw)
        self.MRleg.setYaw(yaw)
        self.FRleg.setYaw(yaw)
        self.BLleg.setYaw(yaw)
        self.MLleg.setYaw(yaw)
        self.FLleg.setYaw(yaw)
    
    def setPitch(self, pitch):
        self.BRleg.setPitch(pitch)
        self.MRleg.setPitch(pitch)
        self.FRleg.setPitch(pitch)
        self.BLleg.setPitch(pitch)
        self.MLleg.setPitch(pitch)
        self.FLleg.setPitch(pitch)
    
    def setRoll(self, roll):
        self.BRleg.setRoll(roll)
        self.MRleg.setRoll(roll)
        self.FRleg.setRoll(roll)
        self.BLleg.setRoll(roll)
        self.MLleg.setRoll(roll)
        self.FLleg.setRoll(roll)
    
    def setBodyPosition(self, bodyPos):
        self.BRleg.setBodyPosition(bodyPos)
        self.MRleg.setBodyPosition(bodyPos)
        self.FRleg.setBodyPosition(bodyPos)
        self.BLleg.setBodyPosition(bodyPos)
        self.MLleg.setBodyPosition(bodyPos)
        self.FLleg.setBodyPosition(bodyPos)
    
    def balance(self):
        imu_pitch = self.imu.getPitch()
        imu_roll = self.imu.getRoll()
        
        self.currentPitch += -imu_roll / 2.0
        self.currentRoll += -imu_pitch / 2.0
        
        self.setRoll(self.currentRoll)
        self.setPitch(self.currentPitch)
    
    def toGround(self):
        self.BRleg.moveToGround(self.currentHeight)
        self.MRleg.moveToGround(self.currentHeight)
        self.FRleg.moveToGround(self.currentHeight)
        self.BLleg.moveToGround(self.currentHeight)
        self.MLleg.moveToGround(self.currentHeight)
        self.FLleg.moveToGround(self.currentHeight)
    
    def startMove(self):
        self.BRleg.startMotor()
        self.MRleg.startMotor()
        self.FRleg.startMotor()
        self.BLleg.startMotor()
        self.MLleg.startMotor()
        self.FLleg.startMotor()