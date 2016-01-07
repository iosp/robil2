# PLP Waypoint Design

This plp was created manually, but designed towards auto-generation. It has the following classes:

## PlpWaypointRosHarness
This is the ROS node. All ROS connections are here. This class mantains a parameters class according the the updated
topics, and waits for the trigger (defined in the PLP document) to happen. When it does, it can create a PlpWaypoint
object, capture the state of the parameters to a file, or both.

## PlpWaypoint
The PLP itself. Can predict the outcome of attempting to follow a path, and monitor the task as it advances. The PLP
does not communicate with ROS directly - it calls it's harness to do so, via a callback mechanism.

## Module PlpAchieveClasses
Contains the outgoing messages from an achieve PLP (prediction, monitoring...)

## PlpWaypointClasses::PlpWaypointParameters
Object containing the PLP parameters, as described by the PLP document. Has a listener mechanism. This is used by the
harness: when the PLP object gets created, it listens to the parameters object maintained by the harness. This way,
whenever the parameters are updated, the PLP can re-evaluate its variables and issue monitoring messages as needed.

## PlpWaypointClasses::PlpWaypointVariables
The variables of the PLP, as defined in the PLP document. The fact that they all live in a single object allows the
PLP to maintain a historical list of them, which is sometimes needed for monitoring (e.g. trend detection).


