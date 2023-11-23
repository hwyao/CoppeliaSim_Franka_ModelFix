classdef testFKM < matlab.unittest.TestCase

    properties
        % vrep (CoppeliaSim) interface
        vi;

        % franka factory handle
        kinematicFactory = FrankaFactory();

        % configurations
        tol = 1e-4;

        % joint limit value (in radius) from franka offical site
        % https://frankaemika.github.io/docs/control_parameters.html#limits-for-panda
        %       -166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027  -166.0031
        q_min = [-2.8973; -1.7628; -2.8973; -3.0718; -2.8973; -0.0175; -2.8973]; 
        %        166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024  166.0031
        q_max = [2.8973; 1.7628; 2.8973; -0.0698; 2.8973; 3.7525; 2.8973];
    end

    properties (TestParameter)
        %%%%%%%%%%%% test value %%%%%%%%%%%%%
        % the value here are all same. But we use this TestParameter to
        % quickly generate and manage differents test combination with the
        % same test function.

        compareTarget = {'VREP'}

        compareBaseline = {'Matlab','DQ'} 
        
        targetFrame = {0, ...
                       1,2,3,4,5,6,7, ...
                       8};
        
        relativeFrame = {0, ...
                         1,2,3,4,5,6,7, ...
                         8};

        includeCurrent = {true,false};
        
        %%%%%%%%%%%% configuration %%%%%%%%%%%%%
        config = struct('home', [0; -0.785398163397; 0; -2.35619449019; 0; 1.57079632679; 0.785398163397], ...
                        'limit',[0; 0; 0; -0.069813170079773; 0; 0; 0], ...
                        'random',[1.1230; 0.5620; 0.5342; -1.4188; -0.5904; 1.2136; 0.9744]);
        % - 'home' is the home configuration of franka.
        %  See: https://github.com/frankaemika/franka_ros/blob/develop/franka_control/config/start_pose.yaml
        % - 'limit' is a configuration that is very similar to the
        % configuration in DH. However, the 4th joint cannot reach
        % that angle due to joint limit.
        %  See: https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
        % - 'random' just a random config. You can refer to
        % https://www.mathworks.com/help/matlab/matlab_prog/use-external-parameters-in-parameterized-test.html
        % to replace it with your random (wished) config
    end
    
    methods(TestClassSetup)
        function establishConnection(testCase)
            % change current folder, if we are in test folder go up one
            % folder. This allows you to run test when you are currently in test folder. 
            currentFolder = pwd;
            if ispc
                currentFolder = split(currentFolder,"\");
            else
                currentFolder = split(currentFolder,"/");
            end
            currentFolder = currentFolder(end);
            if isequal(currentFolder,"test")
                cd('../')
            end

            % establish connection
            testCase.vi = DQ_VrepInterface;
            testCase.vi.disconnect_all();
            testCase.vi.connect('127.0.0.1',19997);
            testCase.vi.start_simulation();
        end

        function testHelperFunction(testCase)
            % test if the function dq2tfrom and tfrom2dq works fine.
            % unit test
            testCase.assumeEqual(tfrom2dq(eye(4)),DQ(1),'AbsTol',testCase.tol);
            testCase.assumeEqual(dq2tfrom(DQ(1)),eye(4),'AbsTol',testCase.tol);

            % example test
            dqExample1 = normalize(DQ([3.2449,-0.7102,-0.8231,1.7251,5.3014,3.9830,2.8176,-3.3226]));
            tfromExample1 = [0.5025   -0.6827   -0.5305    1.9832;
                             0.8419    0.5261    0.1205    2.4535;
                             0.1969   -0.5072    0.8391   -2.5396;
                             0         0         0    1.0000];
            testCase.assumeEqual(vec8(tfrom2dq(tfromExample1)*dqExample1'),vec8(DQ(1)),'AbsTol',testCase.tol);
            testCase.assumeEqual(dq2tfrom(dqExample1),tfromExample1,'AbsTol',testCase.tol);

            % random loopback test
            pos = rand(3,1);
            eul = 2*pi*rand(1,3)-pi;
            tfrom = zeros(4,4);
            tfrom(1:3,1:3) = eul2rotm(eul);
            tfrom(1:3,4) = pos;
            tfrom(4,4) = 1;
            testCase.assumeEqual(dq2tfrom(tfrom2dq(tfrom)),tfrom,'AbsTol',testCase.tol);
        end
    end

    methods(TestClassTeardown)
        % Shared teardown for the entire test class
        function abolishConnection(testCase)
            testCase.vi.stop_simulation();
            testCase.vi.disconnect();
        end
    end
    
    methods(Test)
        function DQ_warningfree(testCase)
            % assert that the are not warning from DQ robotics over the
            % 'DQ_SerialManipulator(DH_matrix,'modified');'
            % If this fail, is it possible that DQ robotics already have an later update
            % and some part of the code is better to be changed.
            testCase.assertWarningFree(@() FrankaDQ())
        end

        function vrep_jointLimit(testCase)
            % Test that vrep model is keeping the correct joint limit. 
            % In simulator, each joint have its joint limit. If we send a
            % value that exceeds the limit, the robot would refuse to go to
            % that angle and still keep the same.
            % If this failed, the joint limit of that model is them
            % problematic. Which might fluence the correctness of later test. 
            FrankaVrepObj = FrankaVrep(testCase.vi);
            
            % assert that the vrep robot should least move inside the range of joint limit
            FrankaVrepObj.set_joint(testCase.q_min);
            q_receive_min = FrankaVrepObj.get_joint();
            testCase.assertEqual(q_receive_min,testCase.q_min,'AbsTol',testCase.tol);

            FrankaVrepObj.set_joint(testCase.q_max);
            q_receive_max = FrankaVrepObj.get_joint();
            testCase.assertEqual(q_receive_max,testCase.q_max,'AbsTol',testCase.tol);
    
            % assert that the vrep robot keeps the joint limit
            q_min_under = testCase.q_min - 0.02;
            FrankaVrepObj.set_joint(q_min_under);
            q_receive_min = FrankaVrepObj.get_joint();
            testCase.assertEqual(q_receive_min,testCase.q_min,'AbsTol',testCase.tol);

            q_max_above = testCase.q_max + 0.02;
            FrankaVrepObj.set_joint(q_max_above);
            q_receive_max = FrankaVrepObj.get_joint();
            testCase.assertEqual(q_receive_max,testCase.q_max,'AbsTol',testCase.tol);
        end
    end
    
    methods(Test)
        % Test single EE
        function EETest(testCase,compareTarget,compareBaseline,config)
            testCase.assumeEqual(config > testCase.q_min & config < testCase.q_max,true(7,1));

            FrankaTarget = testCase.kinematicFactory.construct(compareTarget,testCase.vi);
            FrankaCompare = testCase.kinematicFactory.construct(compareBaseline,testCase.vi);

            dq_target = FrankaTarget.get_EE_pose(config);
            dq_compare = FrankaCompare.get_EE_pose(config);

            [dq_compare,dq_target] = dqCompareHelp(dq_compare,dq_target);

            testCase.assertEqual(vec8(dq_target),vec8(dq_compare),'AbsTol',testCase.tol);
        end
        
        function FrameAbsoluteTest(testCase,compareTarget,compareBaseline,targetFrame,config,includeCurrent)
            testCase.assumeEqual(config > testCase.q_min & config < testCase.q_max,true(7,1));

            FrankaTarget = testCase.kinematicFactory.construct(compareTarget,testCase.vi);
            FrankaCompare = testCase.kinematicFactory.construct(compareBaseline,testCase.vi);

            dq_target = FrankaTarget.get_joint_pose(config,targetFrame,includeCurrent);
            dq_compare = FrankaCompare.get_joint_pose(config,targetFrame,includeCurrent);

            [dq_compare,dq_target] = dqCompareHelp(dq_compare,dq_target);

            testCase.assertEqual(vec8(dq_target),vec8(dq_compare),'AbsTol',testCase.tol);
        end

        % function FrameRelativeTest(testCase,compareTarget,compareBaseline,targetFrame,relativeFrame,config)
        %     testCase.assumeEqual(config > testCase.q_min & config < testCase.q_max,true(7,1));
        % 
        %     FrankaTarget = testCase.kinematicFactory.construct(compareTarget,testCase.vi);
        %     FrankaCompare = testCase.kinematicFactory.construct(compareBaseline,testCase.vi);
        % 
        %     dq_target = FrankaTarget.get_relative_joint_pose(config,targetFrame,relativeFrame);
        %     dq_compare = FrankaCompare.get_relative_joint_pose(config,targetFrame,relativeFrame);
        % 
        %     [dq_compare,dq_target] = dqCompareHelp(dq_compare,dq_target);
        % 
        %     testCase.assertEqual(vec8(dq_target),vec8(dq_compare),'AbsTol',testCase.tol);
        % end
    end
end