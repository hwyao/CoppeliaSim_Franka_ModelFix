function [dq1,dq2] = dqCompareHelp(dq1,dq2)
%DQCOMPAREHELP Because for dual quaterion dq, dq and dq*(-1) actually
%represent the same pose, but expressed differently. Therefore the helper
%function can help we to keep the first term have the same size.
    differenceVec8 = vec8(dq1*dq2');
    if sign(differenceVec8(1)) == -1
        dq2 = dq2*-1;
    end
end

