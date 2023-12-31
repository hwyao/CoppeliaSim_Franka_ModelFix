% Copyright 2023 Haowen Yao
%
% This file is part of the CoppeliaSim_Franka_ModelFix repository.
% 
%     Use of this source code is governed by an MIT-style
%     license that can be found in the LICENSE file or at
%     https://opensource.org/licenses/MIT.

classdef FrankaInterface < handle
    %FRANKAINTERFACE the interface class of Franka that used for the test.

    methods (Abstract)
        dq_out = get_EE_pose(obj,config)
        % get the end-effector expressed in dual quaterion

        dq_out = get_joint_pose(obj,config,num,includeCurrent)
        % get the pose of a specific joint
        % num: which joint we would like to use
        %   num = 0 base
        %   num = 1,2,3,4,5,6,7 different links 
        %   num = 8 end-effector
        % includeCurrent: include the current joint rotation
        %   true = include the current joint rotation (normally work like this)
        %   false = does not include the current joint rotation (to help comparison with VREP)

        dq_out = get_relative_joint_pose(obj,config,linkTarget,linkRelative,includeCurrent)
        % get the relative pose between specific joint
        % linkTarget, linkRelative definition same as above.
        % Because we are testing coppeliaSim (VREP), relative frame is defined according 
        % to the reaction of the VREP remote API.
        % Important: relative pose in VREP is defined differently in
        % compared with other toolboxes.
        % includeCurrent: include the current joint rotation
        %   true = include the current joint rotation (normal assumption)
        %   false = does not include the current joint rotation (VREP assumption)
    end
end

