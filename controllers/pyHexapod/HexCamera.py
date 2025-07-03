from controller import Robot, Camera

class HexCamera:
    def __init__(self, robot, device_name="camera"):
        """
        简化的相机控制器，仅用于显示画面
        :param robot: Webots机器人实例
        :param device_name: 相机设备名称
        """
        # 获取相机设备
        self.camera = robot.getDevice(device_name)
        if not self.camera:
            raise RuntimeError(f"找不到相机设备 '{device_name}'")
            
        self.timestep = int(robot.getBasicTimeStep())
        
        # 启用基础图像采集
        self.camera.enable(self.timestep)
        
        # 图像参数
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        print(f"相机已启用: {self.width}x{self.height}像素")
        
        # 图像数据
        self.image_data = None
    
    def update(self):
        """更新相机图像数据"""
        self.image_data = self.camera.getImage()
    
    def get_image(self):
        """获取当前图像数据"""
        return self.image_data
    
    def get_dimensions(self):
        """获取图像尺寸"""
        return (self.width, self.height)
    
    def cleanup(self):
        """清理资源"""
        self.camera.disable()
        print("相机已禁用")