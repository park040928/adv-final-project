close all;
clear all;
clc;

global gRobotRadius gRobotWheelDist gRobotWheelRadius;
gRobotRadius = 0.2;
gRobotWheelDist = 0.3;
gRobotWheelRadius = 0.1;

[robot,scan,image] = update_robot(0,0);

[robot,scan,image] = update_robot_open(0,0);
