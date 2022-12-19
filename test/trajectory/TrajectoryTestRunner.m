import matlab.unittest.TestRunner
import matlab.unittest.TestSuite
import matlab.unittest.plugins.XMLPlugin

% test specific path
addpath(pwd)

cd('../..')

addpath(genpath('src'))

suite = TestSuite.fromClass(?generateTrapezoidalVelProfileTester);

runner = TestRunner.withNoPlugins;

xmlFile = 'trajectory.xml';
p = XMLPlugin.producingJUnitFormat(xmlFile);

runner.addPlugin(p)
results = runner.run(suite);
table(results)
