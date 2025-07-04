import math
import numpy as np
from controller import Robot, Keyboard
from HexapodRobot import Hexapod
import websockets
import json
import asyncio
import threading
from websockets.asyncio.server import serve


class HexapodController:
    def __init__(self):
        print("Initializing Hexapod Controller...")
        self.robot = Hexapod()
        self.time_step = int(self.robot.getBasicTimeStep())

        # 初始化键盘控制
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.time_step)

        # 初始化参数
        self.omega = 0.0
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.type_key = False

        # 设置初始高度
        self.robot.setHeight(0.100459)
        self.robot.reInit()
        self.robot.startMove()

        # WebSocket相关
        self.websocket_server = None
        self.websocket_thread = None
        self.start_websocket_server()

        self.robot.init_lidar()
        self.camera_send_counter = 0
        self.camera_send_interval = 10  # 每10个时间步发送一次相机数据

    def process_lidar_data(self, points):
        """点云数据处理示例"""
        if len(points) > 0:
            points_arr = np.array([[p.x, p.y, p.z] for p in points if p])
            if points_arr.size > 0:
                front_points = points_arr[
                    (points_arr[:, 1] > 0) & (np.abs(points_arr[:, 0]) < 0.2)
                ]
            
                if front_points.size > 0:
                    closest = front_points[front_points[:, 2].argmin()]
                    print(
                        f"最近障碍物: ({closest[0]:.3f}, {closest[1]:.3f}, {closest[2]:.3f})"
                    )
                    return closest[2]
        return float("inf")

    async def handle_connection(self, websocket):
        print("客户端已连接")
        try:
            async for message in websocket:
                try:
                    command = json.loads(message)
                    # print(message)
                    if command["type"] == "move":

                        print(f"Received move command: {command}")
                        # 在这里处理来自WebSocket的移动命令
                        # 例如更新self.velocity或self.omega
                    elif command["type"] == "lidar":
                        # 发送传感器数据回客户端
                        sensor_data = self.robot.collect_sensor_data()
                        await websocket.send(
                            json.dumps({"sensorData": sensor_data, "status": "ok"})
                        )
                    elif command["type"] == "request_camera":
                    # 立即发送相机数据
                        camera_data = self.robot.get_camera_data()
                        await websocket.send(json.dumps({
                        "type": "camera",
                        "data": camera_data
                    }))
                except json.JSONDecodeError:
                    print("Invalid JSON received")
        except websockets.exceptions.ConnectionClosed:
            print("客户端断开连接")

    def start_websocket_server(self):
        """在单独线程中启动WebSocket服务器"""

        async def server_main():
            async with serve(
                self.handle_connection,
                "0.0.0.0",
                7920,
                max_size=None,  # 增大最大消息大小到 16MB (默认是 1MB)
            ):
                await asyncio.Future()  # 永久运行

        def run_server():
            asyncio.run(server_main())

        self.websocket_thread = threading.Thread(target=run_server, daemon=True)
        self.websocket_thread.start()
        print("WebSocket server started in background thread")

    def run(self):
        """主控制循环"""
        print("Starting main control loop...")
        while self.robot.step(self.time_step) != -1:
            # 处理键盘输入
            self.type_key = False
            velocity = np.array([0.0, 0.0, 0.0])

            # 获取键盘状态
            key = self.keyboard.getKey()

            # 处理移动控制
            if key == ord("A"):
                velocity[0] -= 1.0
                self.type_key = True
            if key == ord("D"):
                velocity[0] += 1.0
                self.type_key = True
            if key == ord("W"):
                velocity[1] += 1.0
                self.type_key = True
            if key == ord("S"):
                velocity[1] -= 1.0
                self.type_key = True

            if self.type_key:
                velocity_magnitude = np.linalg.norm(velocity)
                if velocity_magnitude > 0:
                    self.velocity = velocity / velocity_magnitude * 0.1
            else:
                self.velocity = velocity

            # 处理旋转控制
            omega = 0.0
            if key == ord("Q"):
                omega += 0.2
            if key == ord("E"):
                omega -= 0.2
            if key == ord("R"):
                self.robot.reInit()
                self.robot.startMove()

            self.omega = omega

            # 执行步态
            self.robot.moveTripod()
            
            # 更新相机数据发送计数器
            self.camera_send_counter += 1
            if self.camera_send_counter >= self.camera_send_interval:
                self.camera_send_counter = 0
                
                # 获取相机数据
                camera_data = self.robot.get_camera_data()
                
                # 通过WebSocket发送数据
                if self.websocket_server:
                    message = json.dumps({
                        "type": "camera",
                        "data": camera_data
                    })
                    # 使用异步方式发送
                    asyncio.run(self.websocket_server.broadcast(message))


            # 收集并处理雷达数据
            # sensor_data = self.robot.collect_sensor_data()
            # if 'lidar' in sensor_data:
            #     closest_distance = self.process_lidar_data(sensor_data['lidar']['points'])


def main():
    controller = HexapodController()
    controller.run()


if __name__ == "__main__":
    print("Starting controller...")
    main()
    print("Controller exited")
