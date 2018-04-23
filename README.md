# ANDY

Copyright@RM-Lab.

Andy is a nonholonomic modeling toolbox for hybrid multibody dynamic systems.

Please contact Jiamin Wang (PhD Student at RM-Lab) - jmechw@vt.edu for technical details.

MATLAB Version Requirement: 2017 or later.

## Latest Notification
1. New version available on request, please contact jmechw@vt.edu if you want the latest version of ANDY.

## Examples
To run the examples, please make sure that the library is in the correct relative path with respect to your modeling script, [PathSetup.m] will help you setup the library path profiles. Modify if necessary.

The current examples contains <Passive Walker> and <Swing Bar>
  
For <Passive Walker>: Run [PassiveWalker_Simulation.m] if you are only interested in the 3D animation (Uncomment the PathSetup); Run [PassiveWalker_Modeling.m] only when you aer interested in the modeling process.

For <Swing>: Run Modeling and Simulation in [SwingBar.m].

## Past Notification (04/16/2018)
Future updates will be made consistantly to improve calculation efficiency and include analytical tools.

Next Update (Expected before April 23th) Preview:

1. Add new coding notion that no longer require the matlab 'simplify' function during the generation of dynamics, which greatly reduced calculation time and improves system.

2. Add Example of a Float Based 3D Quadruped Model.
