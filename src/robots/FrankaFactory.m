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
                name(1,:) char {mustBeMember(name,{'Matlab','DQ','VREP'})}
                vi
            end

            if isequal(name,"Matlab")
                FrankaKinematic = FrankaMatlab();
            elseif isequal(name,"DQ")
                FrankaKinematic = FrankaDQ();
            elseif isequal(name,"VREP")
                FrankaKinematic = FrankaVrep(vi);
            end
        end
    end
end

