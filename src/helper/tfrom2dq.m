% Copyright 2023 Haowen Yao
%
% This file is part of the CoppeliaSim_Franka_ModelFix repository.
% 
%     Use of this source code is governed by an MIT-style
%     license that can be found in the LICENSE file or at
%     https://opensource.org/licenses/MIT.

function dq = tfrom2dq(tform)
%DQ2TFROM convert homogenous transformation matrix to dual quaternion
    rotdq = DQ(rotm2quat(tform(1:3,1:3)));
    posdq = DQ(tform(1:3,4));

    dq = rotdq + DQ.E * 0.5 * posdq * rotdq;
end

