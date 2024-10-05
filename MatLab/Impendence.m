clc; 
clear;
close all;

% 参数设置
l1 = 0.237;  % 链接1长度
l2 = 0.340;  % 链接2长度
K = 300 * [1; 1; 1];  % 增加阻抗控制增益
C = 50 * [1; 1; 1];  % 增加阻尼增益
M = [1; 1; 1];        % 惯性参数

% 滑模控制参数
lambda = 1.5;  % 增加滑模增益
k_s = 3;     % 增加控制增益
saturation_limit = 15;  % 调整饱和限制
epsilon = 0.05;  % 较小的边界层厚度

% 初始条件
joint_state_now = [0; 0; 0];  % 当前关节角度 (yaw, pitch1, pitch2)
joint_velocity_now = [0; 0; 0];  % 当前关节速度

% 目标点
end_point_desired = [0.12; 0.12; 0.12];
end_point_now = compute_end_point(joint_state_now, l1, l2)';  % 更新末端位置
end_point_velocity_now = [0; 0; 0];

% 时间参数
dt = 0.01;  % 时间步长
t_end = 5.0;  
t_steps = floor(t_end / dt);

% 初始化存储
end_state_history = zeros(t_steps, 3);
end_velocity_history = zeros(t_steps, 3);  
joint_torque_history = zeros(t_steps, 3);

% 仿真循环
for i = 1:t_steps
    % 计算位置和速度误差
    position_error = end_point_desired - end_point_now;
    velocity_error = -end_point_velocity_now;  

    % 定义滑模面
    s = position_error + lambda * velocity_error;

    % 滑模控制律
    control_input = K .* position_error + C .* velocity_error - k_s * sign(s);
    control_input = control_input .* (abs(s) > epsilon);  % 使用边界层
    control_input = max(min(control_input, saturation_limit), -saturation_limit);
    
    % 运动学计算
    Jabin = compute_jacobian(joint_state_now, l1, l2);

    % 计算扭矩
    joint_torque_now = Jabin' * control_input;  % 使用转置雅可比

    % 更新动态（简单积分）
    joint_velocity_now = joint_velocity_now + (joint_torque_now ./ M) * dt;
    joint_state_now = joint_state_now + joint_velocity_now * dt;

    % 更新末端位置
    end_point_now = compute_end_point(joint_state_now, l1, l2)';  % 更新末端位置
    end_point_velocity_now = Jabin * joint_velocity_now;

    % 存储历史数据
    end_state_history(i, :) = end_point_now';
    end_velocity_history(i, :) = end_point_velocity_now';  
    joint_torque_history(i, :) = joint_torque_now';
end

% 绘制结果
time = (0:dt:t_end-dt)';

figure;
subplot(3, 1, 1);
plot(time, end_state_history);
title('End Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x', 'y', 'z');

subplot(3, 1, 2);
plot(time, end_velocity_history);
title('End Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('vx', 'vy', 'vz');

subplot(3, 1, 3);
plot(time, joint_torque_history);
title('Joint Torque');
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('tau1', 'tau2', 'tau3');

sgtitle('Impedance Control with Sliding Mode Results');

% 计算末端执行器位置
function end_point = compute_end_point(joint_state, l1, l2)
    theta1 = joint_state(1);
    theta2 = joint_state(2);
    theta3 = joint_state(3);
    
    c1 = cos(theta1);
    s1 = sin(theta1);
    c2 = cos(theta2);
    s2 = sin(theta2);
    c23 = cos(theta2 + theta3);
    s23 = sin(theta2 + theta3);

    end_point(1) = c1 * (c2 * l1 + c23 * l2);
    end_point(2) = s1 * (c2 * l1 + c23 * l2);
    end_point(3) = l1 * s2 + l2 * s23;
end

% 计算雅可比矩阵
function J = compute_jacobian(joint_state, l1, l2)
    theta1 = joint_state(1);
    theta2 = joint_state(2);
    theta3 = joint_state(3);
    
    c1 = cos(theta1);
    s1 = sin(theta1);
    c2 = cos(theta2);
    s2 = sin(theta2);
    c23 = cos(theta2 + theta3);
    s23 = sin(theta2 + theta3);

    J = [-s1 * (c2 * l1 + c23 * l2), -c1 * s2 * l1 - c1 * s23 * l2, -c1 * s23 * l2;
          c1 * (c2 * l1 + c23 * l2), -s1 * s2 * l1 - s1 * s23 * l2, -s1 * s23 * l2;
          0, c2 * l2 + c23 * l2, c23 * l2];  % 假设z方向变化
end
