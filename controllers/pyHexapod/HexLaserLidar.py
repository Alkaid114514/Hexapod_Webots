from controller import Lidar

class LaserLidar:
    def __init__(self, device_name):
        self.lidar = Lidar(device_name)
        self.enable(True)  # 默认禁用
        self.lidar.enablePointCloud()
        
    
    def enable(self, sampling_period=32):
        """启用雷达并设置采样周期"""
        self.lidar.enable(sampling_period)
    
    def disable(self):
        self.lidar.disable()
    
    def get_point_cloud(self):
        """获取原始点云数据"""
        return self.lidar.getPointCloud()
    
    def get_range_image(self):
        """获取深度图数据"""
        return self.lidar.getRangeImage()
    
    def get_fov(self):
        """获取视场角参数"""
        return {
            'vertical': self.lidar.getVerticalFov() if self.lidar.getNumberOfLayers() > 1 else 0
        }