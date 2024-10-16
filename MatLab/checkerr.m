clc
clear all

syms theta1 theta2 theta3;
syms c1 s1 c2 s2 c3 s3;
syms l1 l2;

l1 = 0.237;l2 = 0.34;

theta1 = pi/3;theta2 = pi/3;theta3 = -pi/3;
x_1 = cos(theta1)*(cos(theta2)*l1+cos(theta2+theta3)*l2);
y_1 = sin(theta1)*(cos(theta2)*l1+cos(theta2+theta3)*l2);
z_1 = sin(theta2)*l1+sin(theta2+theta3)*l2;

theta1 = pi/3 ;theta2 = pi/3;theta3 = -pi/3+pi/180;
x_2 = cos(theta1)*(cos(theta2)*l1+cos(theta2+theta3)*l2);
y_2 = sin(theta1)*(cos(theta2)*l1+cos(theta2+theta3)*l2);
z_2 = sin(theta2)*l1+sin(theta2+theta3)*l2;

x_2-x_1
y_2-y_1
z_2-z_1