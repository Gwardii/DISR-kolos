import matlab.unittest.TestRunner
import matlab.unittest.TestSuite

% test specific path
addpath(pwd)

cd('../..')

addpath(genpath('src'))

runner = TestRunner.withNoPlugins;

suite = [TestSuite.fromClass(?exampleTask1Tester), ...
             TestSuite.fromClass(?exampleTask2Tester), ...
             TestSuite.fromClass(?exampleTask3Tester), ...
             TestSuite.fromClass(?exampleTask4Tester)];
results = runner.run(suite);
table(results)
