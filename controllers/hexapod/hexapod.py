"""hexapod controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np 

class Hexapod(Robot):
    def __init__(self):
        super(Hexapod, self).__init__()
        self.femur = 79.2/1000
        self.coxa = 38/1000
        self.tibia = 116/1000
        self.bias = {
            "FR": [0.064,0.082,0,0.785398],
            "MR": [0.08,0,0,0],
            "BR": [0.059,-0.083,0,-0.785395],
            "FL": [-0.064,0.082,0,2.3562],
            "ML": [-0.08,0,0,3.14159],
            "BL": [-0.059,-0.083,0,-2.3562]
        }
        
        # self.coxa = 
        self.time_step = int(self.getBasicTimeStep())
        self.loop_counter = 0
        self.motors = {}
        self.p_sensors = {}
        for leg in ["MR","FR","FL","ML","BL","BR"]:
            for seg in ["COXA","FEMUR","TIBIA"]:
                self.motors[leg + '_' + seg] = self.getDevice("M_" + leg + '_' + seg)
                self.p_sensors[leg + '_' + seg] = self.getDevice("S_" + leg + '_' + seg)
                self.p_sensors[leg + '_' + seg].enable(self.time_step)
        self.current_pose = self.init_pose()
        self.set_pose(self.current_pose)
        # dict = {
        #     "FR": {'COXA':0, 'FEMUR':-0.2, 'TIBIA':np.pi/2},
        #     "MR": {'COXA':0, 'FEMUR':-0.9, 'TIBIA':5*np.pi/6},
        #     "BR": {'COXA':0, 'FEMUR':-0.2, 'TIBIA':np.pi/2},
        #     "FL": {'COXA':0, 'FEMUR':-0.9, 'TIBIA':5*np.pi/6},
        #     "ML": {'COXA':0, 'FEMUR':-0.2, 'TIBIA':np.pi/2},
        #     "BL": {'COXA':0, 'FEMUR':-0.9, 'TIBIA':5*np.pi/6}
        # }
        # self.set_pose(dict)
        # targetDict = {
        #     "MR": [0.3 * 3/5,0,-0.05],
        #     "FL": [-0.3 * 2/5,0.1*np.sqrt(3),-0.05],
        #     # "BL": [0,0,0]
        #     "BL": [-0.3 * 2/5,-0.1*np.sqrt(3),-0.05],
        #     # "BR": [0,0,0]
        #     # 'FR': [0,0,0]
        # }
        # angleDict: dict = {

        # }
        # for leg,target in targetDict.items():
        #     angle = self.ik(target,leg)
        
        #     angleDict[leg] = {"COXA": angle[0],"FEMUR": angle[1],"TIBIA": angle[2]}

        # # print(angleDict)
        # self.set_pose(angleDict)
    def walk(self,targetDict):
        angleDict: dict = {

        }
        for leg,target in targetDict.items():
            angle = self.ik(target,leg)
        
            angleDict[leg] = {"COXA": angle[0],"FEMUR": angle[1],"TIBIA": angle[2]}

        # print(angleDict)
        self.set_pose(angleDict)
    
    def generate_tripod_gait(self,step: float,a = 0.05,T = 0.2):
        # isLeftTriangle = True
        targetArr = []
        startTargetArr = []
        linspace = np.linspace(0,T/2.0,int(np.ceil(T/2.0/step)))
        for x in linspace:
            z = a * np.sin(2 * np.pi * x / T) - 0.1
            startTargetArr.append(
                {
                    "MR": [0.3 * 3/5,x, z],
                    "FL": [-0.3 * 2/5,0.1*np.sqrt(3) + x,z],
                    "BL": [-0.3 * 2/5,-0.1*np.sqrt(3) + x,z],
                }
            )
        for x in linspace:
            zl = 0 - 0.1
            z = a * np.sin(2 * np.pi * x / T) - 0.1
            targetArr.append(
                {
                    
                    "MR": [0.3 * 3/5,T/2 - x, zl],
                    "FL": [-0.3 * 2/5,0.1*np.sqrt(3) + T/2 - x,zl],
                    "BL": [-0.3 * 2/5,-0.1*np.sqrt(3) + T/2 - x,zl],
                    
                }
            )
            targetArr.append(
                {
                    "ML": [-0.3 * 3/5,x, z],
                    "FR": [0.3 * 2/5,0.1*np.sqrt(3) + x,z],
                    "BR": [0.3 * 2/5,-0.1*np.sqrt(3) + x,z],
                }
            )
        
        for x in linspace:
            zr = 0 - 0.1
            z = a * np.sin(2 * np.pi * x / T) - 0.1
            targetArr.append(
                {
                    "ML": [-0.3 * 3/5,T/2 - x, zr],
                    "FR": [0.3 * 2/5,0.1*np.sqrt(3) + T/2 - x,zr],
                    "BR": [0.3 * 2/5,-0.1*np.sqrt(3) + T/2 - x,zr],
                    
                }
            )
            
            targetArr.append(
                {
                    "MR": [0.3 * 3/5,x, z],
                    "FL": [-0.3 * 2/5,0.1*np.sqrt(3) + x,z],
                    "BL": [-0.3 * 2/5,-0.1*np.sqrt(3) + x,z],
                }
            )
        return {
            "start": startTargetArr,
            "process": targetArr
        }

    def ik(self, target, leg: str):
        bias = self.bias[leg]
        m = np.zeros((4,4))
        m[:3,:3] = np.array([[np.cos(bias[3]), np.sin(bias[3]), 0],
                            [-np.sin(bias[3]), np.cos(bias[3]), 0],
                            [ 0, 0, 1]])
        m[3,3] = 1
        m[:3,3] = np.array(
            [
             -bias[1] * np.sin(bias[3]) - bias[0] * np.cos(bias[3]),
             -bias[1] * np.cos(bias[3]) + bias[0] * np.sin(bias[3]),
             bias[2]
             ]
             )
        # print(f"矩阵 {m}")
        target = np.array([target[0],target[1],target[2],1])
        target = np.dot(m,target)
        target = [target[0],target[1],target[2]]
        # print(f"腿{leg} 目标点: {target}")
        theta1 = np.atan2(target[1], target[0])
        R = np.sqrt(target[0] ** 2 + target[1] ** 2)
        ar = np.atan2(-target[2], (R - self.coxa))
        Lr = np.sqrt(target[2] ** 2 + (R - self.coxa) ** 2)
        a1 = np.acos(np.clip((self.femur**2 + Lr**2 - self.tibia**2) / (2 * Lr * self.femur), -1, 1))
        theta2 = a1 - ar
        a2 = np.acos(np.clip((Lr**2 + self.tibia**2 - self.femur**2) / (2 * Lr * self.tibia), -1, 1))
        theta3 = -(a1 + a2)
        # if leg.find("L") != -1:
        #     return [-theta1, -theta2, -theta3]
        return [theta1, -theta2, -theta3]
        
    
    def init_pose(self, a=0, b=0.2, c=np.pi/2):
        pose = {}
        for leg in ['MR', 'FR', 'BR','FL', 'ML', 'BL']:
            pose[leg] = {'COXA':a, 'FEMUR':b, 'TIBIA':c} 
        return pose
    

    def set_pose(self, pose: dict):
        # for leg, v in pose.items():
        #     pose[leg]["TIBIA"] -= 1*np.pi/3
        #     for seg,angle in v.items():
        #         self.motors[leg + '_' + seg].setPosition(angle)
        # print(pose)
        for leg,v in pose.items():
            # print(v)
            v['TIBIA'] -= 1*np.pi/3
            for seg in ["COXA","FEMUR","TIBIA"]:
                self.motors[leg + '_' + seg].setPosition(pose[leg][seg])
        # for leg in ['MR', 'FR', 'BR']:
        #     pose[leg]["TIBIA"] -= 1*np.pi/3 
            
        # for leg in ['ML', 'FL', 'BL']:
        #     # pose[leg]["FEMUR"] += 0.44
        #     pose[leg]["TIBIA"] -= 1*np.pi/3
        #     for seg in ["COXA","FEMUR","TIBIA"]:
        #         self.motors[leg + '_' + seg].setPosition(-pose[leg][seg])

    





    def my_step(self):
        if self.step(self.time_step) == -1:
            return 1
        else:
            self.loop_counter += 1
            return 0

if __name__ == "__main__":
    robot = Hexapod()
    robot.my_step()
    index = 0
    tripod = robot.generate_tripod_gait(0.01)
    while not robot.my_step():
        robot.walk(tripod["start"][index])
        index += 1
        if index > len(tripod["start"]) - 1:
            index = 0
            break
    print(tripod["start"])
    # print(tripod["process"])
    while not robot.my_step():
        robot.walk(tripod["process"][index])
        index += 1
        if index > len(tripod["process"]) - 1:
            index = 0