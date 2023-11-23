# CoppeliaSim_Franka_ModelFix

This is a fix over the DH of Franka Emika .ttt model in CoppeliaSim. Alongside with a test-suite to check the consistency between matlab robot toolbox, dqrobotics and CoppeliaSim(V-REP) interface.

Explanation about how the model is modified and when could a problem could appear when you don't repair the model: see [Modification Details](https://github.com/hwyao/CoppeliaSim_Franka_ModelFix/blob/main/docs/modification_details.md).
Explanation about how all these function works: see [Usage Analysis](https://github.com/hwyao/CoppeliaSim_Franka_ModelFix/blob/main/docs/usage_analysis.md)

## Quick Start

Check /model folder to look for the .ttt scene files and find what you need.
Check /docs folder for detailed explanation over all these stuffs.

## Test suite usage

1. Requirements:
- Matlab should at least support testsuite. [Test Browser](https://www.mathworks.com/help/matlab/matlab_prog/run-tests-using-test-browser.html) (after R2023a) is highly recommended for its convenient usage.
- Installing [CoppeliaSim](https://coppeliarobotics.com/downloads) on your machine.
- Installing [DQ robotics](https://github.com/dqrobotics/matlab) in Matlab.
- (optional) Installing Matlab [Robotics System toolbox](https://ww2.mathworks.cn/help/robotics/index.html?s_tid=CRUX_lftnav).
It is suggested to install all components above to run the testsuite comparison. 

1. clone the repository.
```bash
git clone https://github.com/hwyao/CoppeliaSim_Franka_ModelFix
```

3. Copy paste the Coppelia remote API for Matlab in the repository folder.
> remoteApi.\* could be normally found in: 
> \$coppeliaFolder/programming/legacyRemoteApi/remoteApiBindings/lib/lib/\$yourSystem/
> remApi.m and remApiProto could be normally found in: 
> \$coppeliaFolder/programming/legacyRemoteApi/remoteApiBindings/matlab/matlab/

4. Start the CoppeliaSim and load the fixed .ttt scene.

5. Prepare the environment folder
```matlab
prepareEnvironment
```

6. Run the test.
Use the GUI with Test Browser (strongly suggested).
Or run:
```matlab
run(testsuite("testFKM"))
```

## Author's word

If you find this repository helpful, star this repo and share it with the your colleagues working on robotics to help more people!

Contribution of more models / more testsuite based on this fix would be very welcome.