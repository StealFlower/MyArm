classdef RotationMatrix  
    properties  
        Angle % 旋转角度 (弧度)  
        Axis % 旋转轴 (使用 RotationAxis 枚举)  
        Rmat
    end  
    
    methods  
        % 构造函数  
        function obj = RotationMatrix(angle, axis)  
            obj.Angle = angle;  
            obj.Axis = axis;  
            switch obj.Axis  
                case RotationAxis.X  
                    obj.Rmat = [1, 0, 0;  
                         0, cos(obj.Angle), -sin(obj.Angle);  
                         0, sin(obj.Angle), cos(obj.Angle)];  
                case RotationAxis.Y  
                    obj.Rmat = [cos(obj.Angle), 0, sin(obj.Angle);  
                         0, 1, 0;  
                         -sin(obj.Angle), 0, cos(obj.Angle)];  
                case RotationAxis.Z  
                    obj.Rmat = [cos(obj.Angle), -sin(obj.Angle), 0;  
                         sin(obj.Angle), cos(obj.Angle), 0;  
                         0, 0, 1];  
            end  
        end  
    end  
end