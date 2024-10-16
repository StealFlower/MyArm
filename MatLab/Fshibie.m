clc;close all;clear all;
% JointAngle结构体定义  
setAngle_rps = struct('theta1', [], 'theta2', [], 'theta3', []);  
% 假设的输入值  
tarEndPoint = struct('xPos', 1, 'yPos', 1, 'zPos', 0); % 请根据实际情况给tarEndPoint赋值  
mech = struct('l1', 1, 'l2', 1); % 请根据实际情况给mech赋值  
elbm = true; % 也请根据实际情况修改  
PI = pi; % 定义π的值  
THETA2_MECH_TO_RPS_RATIO = 1; % 根据实际情况设置  
THETA3_MECH_TO_RPS_RATIO = 1; % 根据实际情况设置  
Pitch1 = 0; % 假设的Pitch1值  
Pitch2 = 0; % 假设的Pitch2值  

% theta1的逆解  
if abs(tarEndPoint.xPos) < 0.0001 && tarEndPoint.yPos > 0  
    setAngle_rps.theta1 = PI / 2;  
elseif abs(tarEndPoint.xPos) < 0.0001 && tarEndPoint.yPos < 0  
    setAngle_rps.theta1 = -PI / 2;  
else  
    setAngle_rps.theta1 = atan2(tarEndPoint.yPos, tarEndPoint.xPos);  
end  

% theta3的逆解  
if abs((tarEndPoint.xPos^2 + tarEndPoint.yPos^2 + tarEndPoint.zPos^2 - mech.l1^2 - mech.l2^2) / (2 * mech.l1 * mech.l2)) > 0.999  
    setAngle_rps.theta3 = Pitch2 * THETA3_MECH_TO_RPS_RATIO;  
else  
    if elbm  
        setAngle_rps.theta3 = -acos((tarEndPoint.xPos^2 + tarEndPoint.yPos^2 + tarEndPoint.zPos^2 - mech.l1^2 - mech.l2^2) / (2 * mech.l1 * mech.l2));  
    else  
        setAngle_rps.theta3 = acos((tarEndPoint.xPos^2 + tarEndPoint.yPos^2 + tarEndPoint.zPos^2 - mech.l1^2 - mech.l2^2) / (2 * mech.l1 * mech.l2));  
    end  
end  

% theta2的逆解  
c3 = cos(setAngle_rps.theta3);  
s3 = sin(setAngle_rps.theta3);  
if abs(cos(setAngle_rps.theta1)) < 0.1  
    x_c = tarEndPoint.yPos / sin(setAngle_rps.theta1);  
else  
    x_c = tarEndPoint.xPos / cos(setAngle_rps.theta1);  
end  

if abs((x_c * (mech.l1 + c3 * mech.l2) + tarEndPoint.zPos * s3 * mech.l2) / (mech.l1^2 + mech.l2^2 + 2 * c3 * mech.l1 * mech.l2)) > 0.999  
    setAngle_rps.theta2 = (Pitch1 - 90) * THETA2_MECH_TO_RPS_RATIO;  
else  
    setAngle_rps.theta2 = acos((x_c * (mech.l1 + c3 * mech.l2) + tarEndPoint.zPos * s3 * mech.l2) / (mech.l1^2 + mech.l2^2 + 2 * c3 * mech.l1 * mech.l2));  
end  
c1 = cos(setAngle_rps.theta1);  
s1 = sin(setAngle_rps.theta1);  
c2 = cos(setAngle_rps.theta2);  
s2 = sin(setAngle_rps.theta2);  % 输出结果  

Jocobin = [-s1*c2*l1 - s1*c23*l2,-c1*s2*l1-c1*s23*l2,-c1*s23*l2;c1*c2*l1 + c1*c23*l2,-s1*s2*l1-s1*s23*l2,-s1*s23*l2;0,c2*l2+c23*l2,c23*l2];


(transpose(Jocobin))^-1


