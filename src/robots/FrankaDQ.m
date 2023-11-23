classdef FrankaDQ
    %FRANKADQ Franka kinematic implementation with DQ robotics toolbox
    
    properties
        rbt
    end
    
    methods
        function obj = FrankaDQ()
            % The kinematic part is written according to the file
            % commited in https://github.com/dqrobotics/matlab-examples/pull/7
            % which matches the DH parameters publicated by Franka Emika GmbH.
            % https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
            % 
            % DQ_SerialManipulatorMDH (modified/proximal DH) in that file is 
            % still a function under development. It is not included in the 
            % current(20.04.0.1) release of toolbox. 
            % We therefore use the old DQ_SerialManipulator function.
            DH_theta = [0, 0, 0, 0, 0, 0, 0];
            DH_d = [0.333, 0, 0.316, 0, 0.384, 0, 0];
            DH_a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088];
            DH_alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2];
            %DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);

            DH_matrix = [DH_theta;
                         DH_d;
                         DH_a;
                         DH_alpha];
                         %DH_type];
                        
            % The new example in master branch (in rolling development) uses 
            % the 5th row of DH table and removes the convention parameter here. 
            % The current (20.04.0.1) toolbox does not update this feature. 
            % Therefore we keep the old style here.
            % Notice, Franka have 'modified' (distal) DH_parameter here in official document
            % According to commit 279e276, the future update: 
            % does not allow a direct 'DQ_SerialManipulator(A,convention)' construction (Marked as Abstract Now)
            % https://github.com/dqrobotics/matlab/commit/279e276062d4e8ea5c7f11ebac57920c54182404#diff-fafd7e51692a6bd684f91e4bd95c4a822de6299df9b6f3ceba427af4391c30bbR80
            % Also mark the DQ_SerialManipulatorDH(A,convention) as deprecated. (Yeah a good idea to improve the efficiency and readability)
            % https://github.com/dqrobotics/matlab/commit/279e276062d4e8ea5c7f11ebac57920c54182404#diff-c3128d4a1bec4850fe5a40d183b08d1fb6c93e9d3a1fe1268311e6dacdabc07aR175
            % 
            % Notice if your toolbox is higher and updates the feature here.
            obj.rbt = DQ_SerialManipulator(DH_matrix,'modified'); % actually any kind word other than "standard" works
            % kin = DQ_SerialManipulatorMDH(DH_matrix);

            % Set the base's reference frame
            obj.rbt.set_reference_frame(DQ(1));
            obj.rbt.set_base_frame(DQ(1));

            % Set the end-effector shift with flange
            obj.rbt.set_effector(1 + DQ.E*0.5*DQ.k*0.107);
        end
    end

    methods
        function dq_out = get_EE_pose(obj,config)
            dq_out = obj.rbt.fkm(config);
        end

        function dq_out = get_joint_pose(obj,config,num,includeCurrent)
            if includeCurrent == false
                if num>=1 && num<=7
                    config(num) = 0;
                end
            end
            
            if num <= 7
                dq_out = obj.rbt.fkm(config,num);
            elseif num == 8
                dq_out = obj.rbt.fkm(config);
            end
        end

        % function dq_out = get_relative_joint_pose(obj,config,linkTarget,linkRelative)
        %     if linkTarget > linkRelative
        %         if linkTarget>=1 && linkTarget<=7
        %             config(linkTarget) = 0;
        %         end
        % 
        %         if linkTarget <= 7
        %             dq_far = obj.rbt.fkm(config,linkTarget);
        %         else
        %             dq_far = obj.rbt.fkm(config);
        %         end
        %         dq_close = obj.rbt.fkm(config,linkRelative);
        % 
        %         dq_out = dq_far * dq_close';
        % 
        %     elseif linkTarget < linkRelative
        %         if linkTarget>=1 && linkTarget<=7
        %             config(linkTarget) = 0;
        %         end
        % 
        %         dq_close = obj.rbt.fkm(config,linkTarget);
        %         if linkRelative <= 7
        %             dq_far = obj.rbt.fkm(config,linkRelative);
        %         else
        %             dq_far = obj.rbt.fkm(config);
        %         end
        % 
        %         dq_out = dq_close * dq_far';
        % 
        %     else
        %         if linkTarget <= 7
        %             dq_far = obj.rbt.fkm(config,linkTarget);
        %         else
        %             dq_far = obj.rbt.fkm(config);
        %         end
        %         if linkTarget>=1 && linkTarget<=7
        %             config(linkTarget) = 0;
        %         end
        %         if linkTarget <= 7
        %             dq_close = obj.rbt.fkm(config,linkTarget);
        %         else
        %             dq_close = obj.rbt.fkm(config);
        %         end
        % 
        %         dq_out = dq_far * dq_close';
        %     end
        % end
    end
end

