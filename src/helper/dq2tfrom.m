function tform = dq2tfrom(dq)
%DQ2TFROM convert Dual quaternion to homogenous transformation matrix
    tform = zeros(4,4);
    
    rotationVec4 = vec4(dq.rotation);
    tform(1:3,1:3) = quat2rotm(rotationVec4');
    
    traslationVec4 = vec4(dq.translation);
    tform(1:3,4) = traslationVec4(2:4);
    
    tform(4,4) = 1;
end

