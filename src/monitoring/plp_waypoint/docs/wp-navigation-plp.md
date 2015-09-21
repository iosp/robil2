# WP Navigation PLP
- v1 2015-01-12
- v2 2015-01-28
- v3 2015-05-18 Updated to reflect the python code and the recent presentations.
- v4 2015-09-21 Added monitoring, and rearranged "trigger", "monitoring" and "goal" to be under "Lifecycle"

## About
This PLP calculates the success probability of navigating to the end of a local path.

## Values
### Parameters
* `map` &larr; `/PER/MiniMap` occupancy grid of the area. Message is `Map.msg`.
* `path` &larr; `/PP/Path` planned path. Message type is `robil_msgs/Path`
* `position` &larr; `/Loc/Pose` global position. Message type is `PoseWithCovarianceStamped`.
* `positionError` &larr; `/Loc/Pose` (obtained from the covariance component of `position`)
* `mapError` &larr; `TBD` Current assume map error=0. Error will become interesting when we use visual data.

### Variables
* `distanceToWaypoint` local path length to WP
* `mapOccupancy` percentage of occupied cells in MiniMap
* `heightVariablity` height variability in `map`

### Constants
* `MIN_LOC_ERROR` Minimal error under which it is unsafe to drive
* `BOBCAT_SIZE` Size of the bobcat
* `MIN_BLADE_CLEARANCE` Minimal blade clearance for driving
* `FUEL_CONSUMPTION_RATE`
* `BOBCAT_AVERAGE_SPEED`

## Natural Preconditions
* `bladeDof[0] in ([x,y]...) and bladeDof[1] in ([x,y]....) and bladeDof[2] in ([x,y]...)`
    - Ensuring that the IBEO can see the road

## Application Context
### Resources
* Rover (exclusive)

### Preconditions
* `distanceToWaypoint > 1m`
* `positionError < MIN_LOC_ERROR`

### Concurrency Conditions
_Later, we will replace this with "natural" PLP for the entire system. That PLP will maintain an "everything is OK" statement_
* `Bobcat_OK`
* `IBEO_OK`
* `INS_OK`
* `positionError < MIN_LOC_ERROR`
* `Map_OK`
* `bladeClearance > MIN_BLADE_CLEARANCE`

### Concurrent Modules
* blade minimum height preserver

## Side Effects
* Fuel: `FUEL_CONSUMPTION_RATE * distanceToWaypoint * (heightVariablity * hvFactor)`

## Lifecycle (new!)

### Trigger
* Nav task active
* Received local path
* Enough data already gathered (minimap, position, etc.)

### Monitoring
* Remaining path length _(English: Sample `distanceToWaypoint` every 10 seconds. Expect a decrease.)_
  * Every: `10 sec`
  * Expect: `decrease( distanceToWaypoint )`

### Goal
* `distanceToWaypoint < 1m`

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

### Abort
* Getting a new local path

### Pause/Resume
* Pausing and resuming tasks according to the task manager.

### Output
Send a message describing a result to a ROS topic. Exact details TBD.
