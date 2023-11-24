% Copyright 2023 Haowen Yao
%
% This file is part of the CoppeliaSim_Franka_ModelFix repository.
% 
%     Use of this source code is governed by an MIT-style
%     license that can be found in the LICENSE file or at
%     https://opensource.org/licenses/MIT.

function tform = dq2tfrom(dq)
%DQ2TFROM convert Dual quaternion to homogenous transformation matrix
    tform = zeros(4,4);
    
    rotationVec4 = vec4(dq.rotation);
    tform(1:3,1:3) = quat2rotm(rotationVec4');
    
    traslationVec4 = vec4(dq.translation);
    tform(1:3,4) = traslationVec4(2:4);
    
    tform(4,4) = 1;
end

