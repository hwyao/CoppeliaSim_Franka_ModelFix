% Copyright 2023 Haowen Yao
%
% This file is part of the CoppeliaSim_Franka_ModelFix repository.
% 
%     Use of this source code is governed by an MIT-style
%     license that can be found in the LICENSE file or at
%     https://opensource.org/licenses/MIT.

classdef FrankaVrepDQ
    %FRANKAVREPDQ Franka kinematic implementation with DQ_VrepInterface

    properties
        vrepInterface

        vrepJointName = {'/FrankaFix', ...
                         '/FrankaFix/Franka_joint1','/FrankaFix/Franka_joint2',...
                         '/FrankaFix/Franka_joint3','/FrankaFix/Franka_joint4',...
                         '/FrankaFix/Franka_joint5','/FrankaFix/Franka_joint6',...
                         '/FrankaFix/Franka_joint7', ...
                         '/FrankaFix/Franka_connection'};
    end
    
    methods
        function obj = FrankaVrepDQ(vi)
            arguments
                vi DQ_VrepInterface
            end
            obj.vrepInterface = vi;
        end
    end

    methods 
        function dq_out = get_EE_pose(obj,config)
            obj.vrepInterface.set_joint_positions(obj.vrepJointName(2:8),config,obj.vrepInterface.OP_BLOCKING);
            dq_out = obj.vrepInterface.get_object_pose(obj.vrepJointName{9},obj.vrepJointName{1},obj.vrepInterface.OP_BLOCKING);
        end

        function dq_out = get_joint_pose(obj,config,num,includeCurrent)
            obj.vrepInterface.set_joint_positions(obj.vrepJointName(2:8),config,obj.vrepInterface.OP_BLOCKING);
            % This is a interesting characteristics of VREP interface.
            % The default get_object_pose assume() would assme includeCurrent = false
            % i.e. the joint angle of the target link is not included.
            % see description in docs for detail
            if includeCurrent == false
                dq_out = obj.vrepInterface.get_object_pose(obj.vrepJointName{num+1},obj.vrepJointName{1},obj.vrepInterface.OP_BLOCKING);
            else
                dq_out = obj.vrepInterface.get_object_pose(obj.vrepJointName{1},obj.vrepJointName{num+1},obj.vrepInterface.OP_BLOCKING)';
            end
        end

        function dq_out = get_relative_joint_pose(obj,config,linkTarget,linkRelative,includeCurrent)
            obj.vrepInterface.set_joint_positions(obj.vrepJointName(2:8),config,obj.vrepInterface.OP_BLOCKING);
            if includeCurrent == false
                dq_out = obj.vrepInterface.get_object_pose(obj.vrepJointName{linkTarget+1},obj.vrepJointName{linkRelative+1},obj.vrepInterface.OP_BLOCKING);
            else
                dq_tar = obj.vrepInterface.get_object_pose(obj.vrepJointName{1},obj.vrepJointName{linkTarget+1},obj.vrepInterface.OP_BLOCKING)';
                dq_rel = obj.vrepInterface.get_object_pose(obj.vrepJointName{1},obj.vrepJointName{linkRelative+1},obj.vrepInterface.OP_BLOCKING)';
                dq_out = dq_rel' * dq_tar;
            end
        end
    end

    methods
        function set_joint(obj,config)
            obj.vrepInterface.set_joint_positions(obj.vrepJointName(2:8),config,obj.vrepInterface.OP_BLOCKING);
        end

        function config = get_joint(obj)
            config = obj.vrepInterface.get_joint_positions(obj.vrepJointName(2:8),obj.vrepInterface.OP_BLOCKING);
        end
    end
end

