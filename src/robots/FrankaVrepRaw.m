% Copyright 2023 Haowen Yao
%
% This file is part of the CoppeliaSim_Franka_ModelFix repository.
% 
%     Use of this source code is governed by an MIT-style
%     license that can be found in the LICENSE file or at
%     https://opensource.org/licenses/MIT.

classdef FrankaVrepRaw < FrankaInterface
    %FrankaVrepRaw Franka kinematic implementation with official Vrep API
    %implementation
    
    properties
        vi

        vrepJointName = {'/FrankaFix', ...
                         '/FrankaFix/Franka_joint1','/FrankaFix/Franka_joint2',...
                         '/FrankaFix/Franka_joint3','/FrankaFix/Franka_joint4',...
                         '/FrankaFix/Franka_joint5','/FrankaFix/Franka_joint6',...
                         '/FrankaFix/Franka_joint7', ...
                         '/FrankaFix/Franka_connection'};

        handles = [];
    end
    
    methods
        function obj = FrankaVrepRaw(vi)
            obj.vi = vi;

            for i=1:numel(obj.vrepJointName)
                [~,obj.handles(i)] = obj.vi.vrep.simxGetObjectHandle(...
                                    obj.vi.clientID,...
                                    obj.vrepJointName{i},...
                                    obj.vi.OP_BLOCKING);
            end
        end
    end

    methods
        function setConfig(obj,config)
            for iJoint = 1:7
                obj.vi.vrep.simxSetJointPosition(...
                    obj.vi.clientID,...
                    obj.handles(iJoint+1),...
                    config(iJoint),...
                    obj.vi.OP_BLOCKING);
            end
        end

        function t = getPosition(obj,handleTarget,handleRelative)
            [~,t_temp]  = obj.vi.vrep.simxGetObjectPosition(...
                obj.vi.clientID,...
                handleTarget,...
                handleRelative,...
                obj.vi.OP_BLOCKING);
            t_temp = double(t_temp);
            t = DQ([0,t_temp]);
        end

        function r = getRotQuat(obj,handleTarget,handleRelative)
            [~,r_temp] = obj.vi.vrep.simxGetObjectQuaternion( ...
                obj.vi.clientID, ...
                handleTarget,...
                handleRelative,...
                obj.vi.OP_BLOCKING);
            r_temp = double(r_temp);
            r = normalize(DQ([r_temp(4),r_temp(1),r_temp(2),r_temp(3)]));
        end
    end

    methods
        function dq_out = get_EE_pose(obj,config)
            obj.setConfig(config);

            t = obj.getPosition(obj.handles(9),-1);
            r = obj.getRotQuat(obj.handles(9),-1);

            dq_out = r + 0.5*DQ.E*t*r;
        end

        function dq_out = get_joint_pose(obj,config,num,includeCurrent)
            obj.setConfig(config);
            if includeCurrent == false
                t = obj.getPosition(obj.handles(num+1),obj.handles(1));
                r = obj.getRotQuat(obj.handles(num+1),obj.handles(1));
                dq_out = r + 0.5*DQ.E*t*r;
            else
                t = obj.getPosition(obj.handles(1),obj.handles(num+1));
                r = obj.getRotQuat(obj.handles(1),obj.handles(num+1));
                dq_out = r + 0.5*DQ.E*t*r;
                dq_out = dq_out';
            end
        end

        function dq_out = get_relative_joint_pose(obj,config,linkTarget,linkRelative,includeCurrent)
            obj.setConfig(config);
            if includeCurrent == false
                t = obj.getPosition(obj.handles(linkTarget+1),obj.handles(linkRelative+1));
                r = obj.getRotQuat(obj.handles(linkTarget+1),obj.handles(linkRelative+1));
    
                dq_out = r + 0.5*DQ.E*t*r;
            else
                t_tar = obj.getPosition(obj.handles(1),obj.handles(linkTarget+1));
                r_tar = obj.getRotQuat(obj.handles(1),obj.handles(linkTarget+1));
                dq_tar = r_tar + 0.5*DQ.E*t_tar*r_tar;
                dq_tar = dq_tar';

                t_rel = obj.getPosition(obj.handles(1),obj.handles(linkRelative+1));
                r_rel = obj.getRotQuat(obj.handles(1),obj.handles(linkRelative+1));
    
                dq_rel = r_rel + 0.5*DQ.E*t_rel*r_rel;
                dq_rel = dq_rel';

                dq_out = dq_rel' * dq_tar;
            end
        end
    end
end

