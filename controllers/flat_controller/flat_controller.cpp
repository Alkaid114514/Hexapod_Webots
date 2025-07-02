// File:          flat_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <algorithm>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <random> 
// All the webots classes are defined in the "webots" namespace
using namespace webots;
// 球面线性插值 (Slerp)
void slerp(const double start[4], double end[4], double t, double result[4]) {
  double cosHalfTheta = start[0] * end[0] + start[1] * end[1] + start[2] * end[2] + start[3] * end[3];

  // 如果 cosHalfTheta < 0，反转一个四元数以选择最短路径
  if (cosHalfTheta < 0) {
    for (int i = 0; i < 4; ++i) end[i] = -end[i];
    cosHalfTheta = -cosHalfTheta;
  }

  // 如果 start 和 end 非常接近，直接线性插值
  if (std::abs(cosHalfTheta) >= 1.0) {
    for (int i = 0; i < 4; ++i) result[i] = start[i];
    return;
  }

  double halfTheta = std::acos(cosHalfTheta);
  double sinHalfTheta = std::sqrt(1.0 - cosHalfTheta * cosHalfTheta);

  // 如果角度非常小，直接线性插值
  if (std::abs(sinHalfTheta) < 0.001) {
    for (int i = 0; i < 4; ++i) result[i] = start[i] * (1 - t) + end[i] * t;
    return;
  }

  double ratioA = std::sin((1 - t) * halfTheta) / sinHalfTheta;
  double ratioB = std::sin(t * halfTheta) / sinHalfTheta;

  for (int i = 0; i < 4; ++i) result[i] = start[i] * ratioA + end[i] * ratioB;
}
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  // Robot *robot = new Robot();
  Supervisor *supervisor = new Supervisor();

  // get the time step of the current world.
  int timeStep = (int)supervisor->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  Node *robotNode = supervisor->getFromDef("flat123");
  Field *ro = robotNode->getField("rotation");
  
  std::random_device rd;  // 随机设备
  std::mt19937 gen(rd()); // 随机数生成器
  std::uniform_real_distribution<> angleDist(-0.4f, +0.4f); // 角度范围
  std::uniform_real_distribution<> axisDist(-1.0f, 1.0f);  // 轴范围

  // 初始旋转状态
  double currentRotation[4] = {0.0, 0.0, 1.0, 0.0}; // [x, y, z, angle]
  double targetRotation[4] = {0.0, 0.0, 1.0, 0.0};  // [x, y, z, angle]

  // 旋转速度 (弧度/秒)
  double rotationSpeed = 0.02; // 1 弧度/秒

  // 插值参数
  double interpolationFactor = 0.0; // 插值因子 (0 到 1)
  const double interpolationStep = 0.03; // 插值步长

  
  while (supervisor->step(timeStep) != -1) {
    // 如果插值完成，生成新的目标旋转状态
    if (interpolationFactor >= 1.0) {
      interpolationFactor = 0.0;

      // 生成新的目标旋转状态
      double angle = angleDist(gen); // 随机角度 (0 到 2π)
      double axisX = axisDist(gen);  // 随机轴 X (-1 到 1)
      double axisY = axisDist(gen);  // 随机轴 Y (-1 到 1)
      double axisZ = axisDist(gen);  // 随机轴 Z (-1 到 1)

      // 归一化旋转轴
      double length = std::sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
      targetRotation[0] = axisX / length;
      targetRotation[1] = axisY / length;
      targetRotation[2] = axisZ / length;
      targetRotation[3] = angle;
    }

    // 插值当前旋转状态
    double interpolatedRotation[4];
    slerp(currentRotation, targetRotation, interpolationFactor, interpolatedRotation);

    // 更新当前旋转状态
    for (int i = 0; i < 4; ++i) currentRotation[i] = interpolatedRotation[i];

    // 设置新的 rotation 值
    ro->setSFRotation(currentRotation);

    // 增加插值因子
    interpolationFactor += interpolationStep;
  }

  delete supervisor;
  return 0;
  
}
