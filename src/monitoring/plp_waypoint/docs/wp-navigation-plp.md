# WP Navigation PLP

1. 2015-01-12
1. 2015-01-28
1. 2015-05-18 Updated to reflect the python code and the recent presentations.
1. 2015-09-21 Added monitoring, and rearranged "trigger", "monitoring" and "goal" to be under "Lifecycle"
1. 2015-10-07 Restructured, "lifecycle" removed.
1. 2015-11-22 Added new advancement measurement
1. 2015-11-22 Monitoring constants implicitly defines history frame count.

## About
This PLP calculates the success probability of navigating to the end of a local path.

## Values
### Parameters
* `map` &larr; `/PER/MiniMap` occupancy grid of the area. Message is `Map.msg`.
* `path` &larr; `/PP/Path` planned path. Message type is `robil_msgs/Path`
* `position` &larr; `/Loc/Pose` global position. Message type is `PoseWithCovarianceStamped`.
* `positionError` &larr; `/Loc/Pose` (obtained from the covariance component of `position`)
* `mapError` &larr; `TBD` Current assume map error=0. Error will become interesting when we use visual data.

### Constants
* `MIN_LOC_ERROR` Minimal error under which it is unsafe to drive
* `BOBCAT_SIZE` Size of the bobcat
* `MIN_BLADE_CLEARANCE` Minimal blade clearance for driving
* `FUEL_CONSUMPTION_RATE`
* `BOBCAT_AVERAGE_SPEED`
* `RATE_PATH_LENGTH` Rate of expected decrease in the path length, during advancement measurement.
* `RATE_AERIAL_DISTANCE` Rate of expected decrease in the aerial distance to the waypoint, during advancement measurement.

### Variables
_Calculated based on parameters and constants._

* `distanceToWaypoint` local path length to WP
* `mapOccupancy` percentage of occupied cells in MiniMap
* `heightVariablity` height variability in `map`
* `aerialDistanceToWaypoint` distance from bobcat to waypoint

## Natural Preconditions
- Ensuring that the IBEO can see the road
  * `bladeDof[0] in ([x,y]...) and bladeDof[1] in ([x,y]....) and bladeDof[2] in ([x,y]...)`

## Goal
* `distanceToWaypoint < 1m`

## Application Context
### Resources
* Rover (exclusive)

### Preconditions
* `distanceToWaypoint > 1m`
* `positionError < MIN_LOC_ERROR`

### Concurrency Conditions
_Later, we will replace this with "health" PLP for the entire system_

* `Bobcat_OK`
* `IBEO_OK`
* `INS_OK`
* `positionError < MIN_LOC_ERROR`
* `Map_OK`
* `bladeClearance > MIN_BLADE_CLEARANCE`

### Concurrent Modules
* blade minimum height preserver

### Advancement measures
_Defined measurements require 2 frames of variables. This is caputred in the `PLP_WAYPOINT_HISTORY_LENGTH` constant._

* Remaining path length _(English: Sample `distanceToWaypoint` every time the path is published. Expect a decrease.)_
  * Every: `path` publication
  * Expect: `decrease( distanceToWaypoint, RATE_PATH_LENGTH )`
* Distance to Target _(English: Sample `aerialDistanceToWaypoint` every time the path is published. Expect a decrease.)_
  * Every: `path` publication
  * Expect: `decrease( aerialDistanceToWaypoint, RATE_AERIAL_DISTANCE )`

## Side Effects
* Fuel: `FUEL_CONSUMPTION_RATE * distanceToWaypoint * (heightVariablity * hvFactor)`


## Result
_The calculation result describes the probabilities and time for each known outcome: success, and various types of failures._

* success
  * probability: `(explicit subset of the PLP parameters)`
  * time
* failures
  * `Bobcat_stuck`:
    * probability: `(explicit subset of the PLP parameters)`
    * time: `(explicit subset of the PLP parameters)`
  * `Bobcat_turned_over`:
    * probability: `(explicit subset of the PLP parameters)`
    * time: `(explicit subset of the PLP parameters)`
* confidence `A number in the range of 0..1`

## Robil Integration

### Trigger
* Nav task active
* Received local path
* Enough data already gathered (minimap, position, etc.)

### Abort
* Getting a new local path
* Task being paused/aborted

### Output
* Predictions and advance measure warnings sent to the `plp/messages` ROS topic.
