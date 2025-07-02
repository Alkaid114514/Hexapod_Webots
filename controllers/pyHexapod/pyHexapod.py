import math
import numpy as np
from controller import Robot, Keyboard
from HexapodRobot import Hexapod  # 假设Hexapod类已经转换为Python

def main():
    # 创建机器人实例
    robot = Hexapod()
    time_step = int(robot.getBasicTimeStep())

    # 初始化键盘控制
    keyboard = robot.getKeyboard()
    keyboard.enable(time_step)
    
    # 初始化参数
    robot.omega = 0.0
    robot.velocity = np.array([0.0, 0.0, 0.0])

    # 设置初始高度
    robot.setHeight(0.100459)
    robot.reInit()
    robot.startMove()

    type_key = False
    
    while robot.step(time_step) != -1:
        # 处理键盘输入
        type_key = False
        velocity = np.array([0.0, 0.0, 0.0])
        
        # 获取键盘状态
        key = keyboard.getKey()
        
        # 处理移动控制
        if key == ord('A'):
            velocity[0] -= 1.0
            type_key = True
        if key == ord('D'):
            velocity[0] += 1.0
            type_key = True
        if key == ord('W'):
            velocity[1] += 1.0
            type_key = True
        if key == ord('S'):
            velocity[1] -= 1.0
            type_key = True
            
        if type_key:
            velocity_magnitude = np.linalg.norm(velocity)
            if velocity_magnitude > 0:
                robot.velocity = velocity / velocity_magnitude * 0.1
        else:
            robot.velocity = velocity
        
        # 处理旋转控制
        omega = 0.0
        if key == ord('Q'):
            omega += 0.2
        if key == ord('E'):
            omega -= 0.2
        if key == ord('R'):
            robot.reInit()
            robot.startMove()
            
        robot.omega = omega
        
        # 执行步态
        robot.moveTripod()
        # robot.moveRipple()
        # robot.moveWave()
        # robot.move4plus2()
        
        # 可选：平衡控制
        # robot.balance()

if __name__ == "__main__":
    main()