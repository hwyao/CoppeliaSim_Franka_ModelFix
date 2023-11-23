function dq = tfrom2dq(tform)
%DQ2TFROM convert homogenous transformation matrix to dual quaternion
    rotdq = DQ(rotm2quat(tform(1:3,1:3)));
    posdq = DQ(tform(1:3,4));

    dq = rotdq + DQ.E * 0.5 * posdq * rotdq;
end

