# CoppeliaSim_Franka_ModelFix

This repository provides a fix for Franka Emika model in CoppeliaSim. It also includes a test suite to ensure consistency between Matlab Robotics System toolbox, dqrobotics toolbox, CoppeliaSim(V-REP) interface provided by dqrobotics, and the official CoppeliaSim API interface.

For more details about the modifications made to the model and the potential problems that could arise if the model is not repaired, please refer to the [Modification Details](https://github.com/hwyao/CoppeliaSim_Franka_ModelFix/blob/main/docs/modification_details.md).    
Additionally, there is a detailed explanation of key points to consider when using the API in the [Usage Analysis](https://github.com/hwyao/CoppeliaSim_Franka_ModelFix/blob/main/docs/usage_analysis.md).

## Quick Start

Navigate to "/model" folder to look for the `.ttt` scene or `.ttm` model and pick the one you need.   
Navigate to "/docs" folder for further information over all these stuffs.    
Navigate to "/src/robots" folder to see classes that implement the same reaction with different functions. 

## Test suite usage

1. Requirements:
- Matlab should at least support class-based unit test. [Test Browser](https://www.mathworks.com/help/matlab/matlab_prog/run-tests-using-test-browser.html) (after R2023a) is highly recommended for its convenient usage.
- Installing [CoppeliaSim](https://coppeliarobotics.com/downloads) on your machine.
- Installing [DQ robotics](https://github.com/dqrobotics/matlab) in Matlab.
- (optional) Installing Matlab [Robotics System toolbox](https://ww2.mathworks.cn/help/robotics/index.html?s_tid=CRUX_lftnav). 

2. clone the repository.
```bash
git clone https://github.com/hwyao/CoppeliaSim_Franka_ModelFix
```

3. Copy paste the Coppelia remote API for Matlab in the repository folder.
> `remoteApi.*` could be found in:     
> \$coppeliaFolder/programming/legacyRemoteApi/remoteApiBindings/lib/lib/\$yourSystem/
> 
> `remApi.m` and `remApiProto.m` could be found in:     
> \$coppeliaFolder/programming/legacyRemoteApi/remoteApiBindings/matlab/matlab/

4. Start the CoppeliaSim and load the fixed `.ttt` scene.

5. Prepare the environment folder.
```matlab
prepareEnvironment
```

6. Run the test.     
Use the GUI with Test Browser (strongly suggested).     
Or run:
```matlab
run(testsuite("testFKM"))  % run the full fkm test
```
Or edit and run:
```matlab
testFilter  % run the filtered test. You can select the part you want.
```
> Tips:    
> - If you want to use the raw interface instead of the one provided by dqrobotics, you may meet:
> ```matlab
> ERROR: No public property 'vrep' for class 'DQ_VrepInterface'.
> ```
>  You need to set the `vrep` property of the `DQ_VrepInterface` class to public. This modification will also be useful for your other research. 
> ```matlab
> edit DQ_VrepInterface % open the file DQ_VrepInterface.m of dqrobotics toolbox
> % line 81: (Access = private) change to (Access = public)
> ```
> - To understand the code in `testFilter`, you could reference to: [Create suite of tests - MATLAB testsuite](https://www.mathworks.com/help/matlab/ref/testsuite.html) and [matlab.unittest.selectors Package](https://www.mathworks.com/help/matlab/ref/matlab.unittest.selectors-package.html)

## Author's word

If you find this repository helpful, please consider starring it and sharing it with your colleagues working on robotics. This would help us reach and assist more people.

We also encourage further discussions, contributions, and integrations. Your input is highly valued and welcome. 