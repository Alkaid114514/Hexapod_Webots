from controller import Robot, Camera
import numpy as np
from PIL import Image
import base64
import io

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
        self.bpp = 4  # Webots相机返回BGRA格式 (4字节/像素)
        print(f"相机已启用: {self.width}x{self.height}像素")
        
        # 图像数据
        self.image_data = None
        self.raw_bytes = None
    
    def update(self):
        """更新相机图像数据"""
        self.raw_bytes = self.camera.getImage()
        if self.raw_bytes:
            # 使用NumPy高效存储图像数据
            self.image_data = np.frombuffer(self.raw_bytes, dtype=np.uint8)
    
    def get_jpeg_bytes(self, quality=85):
        """
        将当前图像转换为JPEG字节流
        :param quality: JPEG质量 (1-100)
        :return: JPEG字节流或None
        """
        if not self.raw_bytes:
            return None
            
        # 创建PIL图像 (BGRA格式)
        bgra_image = Image.frombytes("RGBA", (self.width, self.height), self.raw_bytes)
        
        # 转换为RGB
        rgb_image = bgra_image.convert("RGB")
        
        # 保存为JPEG字节流
        buffer = io.BytesIO()
        rgb_image.save(buffer, format="JPEG", quality=quality)
        return buffer.getvalue()
    
    def get_base64_jpeg(self, quality=85):
        """
        获取Base64编码的JPEG图像
        :param quality: JPEG质量 (1-100)
        :return: Base64字符串或None
        """
        jpeg_bytes = self.get_jpeg_bytes(quality)
        if not jpeg_bytes:
            return None
            
        return base64.b64encode(jpeg_bytes).decode('ascii')
    
    def get_dimensions(self):
        """获取图像尺寸"""
        return (self.width, self.height)
    
    def cleanup(self):
        """清理资源"""
        self.camera.disable()
        print("相机已禁用")