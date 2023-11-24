% Copyright 2023 Haowen Yao
%
% This file is part of the CoppeliaSim_Franka_ModelFix repository.
% 
%     Use of this source code is governed by an MIT-style
%     license that can be found in the LICENSE file or at
%     https://opensource.org/licenses/MIT.

%% generate the full testsuite
suite = matlab.unittest.TestSuite.fromClass(?testFKM);

% inject the custom configuration as input
% config = {[1.1230; 0.5620; 0.5342; -1.4188; -0.5904; 1.2136; 0.9744]};
% param = matlab.unittest.parameters.Parameter.fromData("config",config);
% suite = matlab.unittest.TestSuite.fromClass(?testFKM,"ExternalParameters",param);

%% filter the need
% if you use many filters, consider change
% ParameterCombination = "pairwise" into "exhaustive" in testCase
import matlab.unittest.selectors.HasProcedureName;
import matlab.unittest.selectors.HasParameter

%suite = suite.selectIf(HasProcedureName("EETest"));

%suite = suite.selectIf(HasProcedureName("FrameAbsoluteTest"));

%suite = suite.selectIf(HasProcedureName("FrameRelativeTest"));

%suite = suite.selectIf(HasProcedureName("EETest") | HasProcedureName("FrameAbsoluteTest"));

suite = suite.selectIf(HasParameter("Property","compareTarget","Name","VREP_DQ"));

suite = suite.selectIf(HasParameter("Property","compareBaseline","Name","DQ"));

suite = suite.selectIf(HasParameter("Property","includeCurrent","Name","true"));


%% display
{suite.Name}'      %#ok<NOPTS>
disp("Press Any Key to Continue...");
pause


%% run and display the result
results = suite.run;
table(results)