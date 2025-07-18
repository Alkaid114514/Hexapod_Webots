import math
import numpy as np
from controller import Robot, Keyboard
from HexapodRobot import Hexapod  # 假设Hexapod类已经转换为Python
from websockets.sync.server import serve
import websockets
import json


def process_lidar_data(points):
    """点云数据处理示例"""
    if len(points) > 0:
        # 转换为NumPy数组处理
        points_arr = np.array([[p.x, p.y, p.z] for p in points if p])
        if points_arr.size > 0:
            # 计算最近点 (仅关注前方区域)
            front_points = points_arr[(points_arr[:,1] > 0) & (np.abs(points_arr[:,0]) < 0.2)]
            if front_points.size > 0:
                closest = front_points[front_points[:,2].argmin()]
                print(f"最近障碍物: ({closest[0]:.3f}, {closest[1]:.3f}, {closest[2]:.3f})")
                return closest[2]  # 返回最近距离
    return float('inf')



def handle_connection(websocket, path):
    print("客户端已连接")
    try:
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

            # 收集并处理雷达数据
            sensor_data = robot.collect_sensor_data()
            if 'lidar' in sensor_data:
                closest_distance = process_lidar_data(sensor_data['lidar']['points'])

            # 处理键盘输入
            type_key = False
            velocity = np.array([0.0, 0.0, 0.0])
            
            try:
                command_str = websocket.recv()
                command = json.loads(command_str)
                if command["type"] == "move":
                    print(f"控制机器人: {command['direction']}, 速度: {command['speed']}")
            except:
                pass  # 无新指令

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
                
                # 发送数据给客户端
            websocket.send(f"DIST:{1}")  # 示例：只发送距离
                
                # 接收客户端指令（非阻塞方式）
            
    except websockets.exceptions.ConnectionClosed:
        print("客户端断开连接")

# def main():
#     # 创建机器人实例
#     robot = Hexapod()
#     time_step = int(robot.getBasicTimeStep())

#     # 初始化键盘控制
#     keyboard = robot.getKeyboard()
#     keyboard.enable(time_step)
    
#     # 初始化参数
#     robot.omega = 0.0
#     robot.velocity = np.array([0.0, 0.0, 0.0])

#     # 设置初始高度
#     robot.setHeight(0.100459)
#     robot.reInit()
#     robot.startMove()

#     type_key = False
    
#     while robot.step(time_step) != -1:
#         # 处理键盘输入
#         type_key = False
#         velocity = np.array([0.0, 0.0, 0.0])
        
#         # 获取键盘状态
#         key = keyboard.getKey()
        
#         # 处理移动控制
#         if key == ord('A'):
#             velocity[0] -= 1.0
#             type_key = True
#         if key == ord('D'):
#             velocity[0] += 1.0
#             type_key = True
#         if key == ord('W'):
#             velocity[1] += 1.0
#             type_key = True
#         if key == ord('S'):
#             velocity[1] -= 1.0
#             type_key = True
            
#         if type_key:
#             velocity_magnitude = np.linalg.norm(velocity)
#             if velocity_magnitude > 0:
#                 robot.velocity = velocity / velocity_magnitude * 0.1
#         else:
#             robot.velocity = velocity
        
#         # 处理旋转控制
#         omega = 0.0
#         if key == ord('Q'):
#             omega += 0.2
#         if key == ord('E'):
#             omega -= 0.2
#         if key == ord('R'):
#             robot.reInit()
#             robot.startMove()
            
#         robot.omega = omega
        
#         # 执行步态
#         robot.moveTripod()
#         # robot.moveRipple()
#         # robot.moveWave()
#         # robot.move4plus2()
        
#         # 可选：平衡控制
#         # robot.balance()

def main():
    with serve(handle_connection, "0.0.0.0", 7920) as server:
        server.serve_forever()
        print(2)


if __name__ == "__main__":
    print(1)
    # main()
    print(3)
