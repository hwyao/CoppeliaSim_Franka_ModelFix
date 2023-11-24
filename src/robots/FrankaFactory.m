% Copyright 2023 Haowen Yao
%
% This file is part of the CoppeliaSim_Franka_ModelFix repository.
% 
%     Use of this source code is governed by an MIT-style
%     license that can be found in the LICENSE file or at
%     https://opensource.org/licenses/MIT.

classdef FrankaFactory
    %FRANKAFACTORY the factory class that construct the different 
    
    methods
        function obj = FrankaFactory()
        end
    end

    methods (Static)    
        function FrankaKinematic = construct(name,vi)
            %CONSTRUCT construct kinematic model with different implementation
            arguments
                name(1,:) char {mustBeMember(name,{'Matlab','DQ','VREP_DQ','VREP_Raw'})}
                vi
            end

            if isequal(name,"Matlab")
                FrankaKinematic = FrankaMatlab();
            elseif isequal(name,"DQ")
                FrankaKinematic = FrankaDQ();
            elseif isequal(name,"VREP_DQ")
                FrankaKinematic = FrankaVrepDQ(vi);
            elseif isequal(name,"VREP_Raw")
                FrankaKinematic = FrankaVrepRaw(vi);
            end
        end
    end
end

