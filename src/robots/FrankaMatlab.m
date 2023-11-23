classdef FrankaMatlab < FrankaInterface
    %FRANKAMATLAB the Franka kinematic implementation with Matlab Robotics
    %Toolbox
    
    properties
        rbt

        allJointName =  {'panda_link0', ...
                         'panda_link1','panda_link2','panda_link3','panda_link4', ...
                         'panda_link5','panda_link6','panda_link7', ...
                         'panda_link8'};
    end
    
    methods
        function obj = FrankaMatlab()
            obj.rbt = loadrobot("frankaEmikaPanda","DataFormat","column");
            obj.rbt.removeBody('panda_hand');
        end
    end

    methods
        function dq_out = get_EE_pose(obj,config)
            dq_out = tfrom2dq(obj.rbt.getTransform(config,obj.allJointName{9}));
        end

        function dq_out = get_joint_pose(obj,config,num,includeCurrent)
            if includeCurrent == false
                if num>=1 && num<=7
                    config(num) = 0;
                end
            end

            dq_out = tfrom2dq(obj.rbt.getTransform(config,obj.allJointName{num+1}));
        end

        % function dq_out = get_relative_joint_pose(obj,config,linkTarget,linkRelative)
        %     if linkTarget > linkRelative
        %         if linkTarget>=1 && linkTarget<=7
        %             config(linkTarget) = 0;
        %         end
        %         dq_out = tfrom2dq(obj.rbt.getTransform(config,obj.allJointName{linkTarget+1},obj.allJointName{linkRelative+1}));
        %     elseif linkTarget < linkRelative
        %         if linkRelative>=1 && linkRelative<=7
        %             config(linkRelative) = 0;
        %         end
        %         dq_out = tfrom2dq(obj.rbt.getTransform(config,obj.allJointName{linkTarget+1},obj.allJointName{linkRelative+1}));
        %     else
        %         dq_after = tfrom2dq(obj.rbt.getTransform(config,obj.allJointName{linkTarget+1},obj.allJointName{linkRelative+1}));
        %         if linkRelative>=1 && linkRelative<=7
        %             config(linkRelative) = 0;
        %         end
        %         dq_before = tfrom2dq(obj.rbt.getTransform(config,obj.allJointName{linkTarget+1},obj.allJointName{linkRelative+1}));
        %         dq_out = dq_after * dq_before';
        %     end
        % end
    end
end

