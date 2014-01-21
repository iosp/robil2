ROBIL2 FRAMEWORK
================

The robil2conf pacakge contains launch scripts and a configuration yaml file.
The configuration yaml contains the topic list for subscribing and publishing for each node.

Launching FrameworkInit.launch will load the parameters file and all the framework nodes.

If one wishes to test a single node he still needs to load the parameter file by running roslaunch robil2conf parametersLoad.launch
and only then run the desired node.
