# WP Navigation PLP
- v1 2015-01-12
- v2 2015-01-28

## About
This PLP calculates the success probability of navigating to the next waypoint in a path.

## Values
### Parameters
* `map` &larr; `/PER/MiniMap` occupancy grid of the area. Message is `Map.msg`.
* `mapError` &larr; `TBD` Current assume map error=0. Error will become interesting when we use visual data.
* `path` &larr; `/PP/Path` planned path
* `position` &larr; `/Loc/Pose` global position. Message type is `PoseWithCovarianceStamped`.
* `positionError` &larr; `/Loc/Pose`
    
### Variables
* `distanceToWaypoint` local path length to WP 
* `mapOccupancy` percentage of occupied cells in MiniMap
* `heightVariablity` height variability in `map`

### Constants
* `MIN_LOC_ERROR` Minimal error under which it is unsafe to drive
* `BOBCAT_SIZE` Size of the bobcat
* `MIN_BLADE_CLEARANCE` Minimal blade clearance for driving

## Robil Integration
### Trigger
* Nav task active
* received local path

### Output 
* `LocSuc` &rarr; `/Some/queue/for/PLPs`  Local Navigation Success Probability
* `LocTime` &rarr; `/Some/queue/for/PLPs` Local Navigation Time Estimation
* `Confidence` &rarr; `/Some/queue/for/PLPs` Confidence level for the estimations given.

## ? Natural Preconditions ?
_Replace with natural PLP for the entire system?_

* `bladeDof[0] in ([x,y]...) and bladeDof[1] in ([x,y]....) and bladeDof[2] in ([x,y]...)`
    - Ensuring that the IBEO can see the road

## Application Context
### Resources
* Rover (exclusive) 

### Preconditions
* `distanceToWaypoint > 1m`
* `clearance(t)+positionError < MIN_LOC_ERROR`

### Concurrency Conditions
* `Bobcat_OK`
* `IBEO_OK`
* `INS_OK`
* `positionError < MIN_LOC_ERROR`
* `Map_OK`
* `bladeClearance > MIN_BLADE_CLEARANCE`

### Concurrent Modules
* blade minimum height preserver

## Side Effects
* Fuel: `fuelConsumptionRate * distanceToWaypoint * (heightVariablity * hvFactor)`

## Goal
* `distanceToWaypoint < 1m`


## Probabilities
* Success: `P_s(explicit subset of the PLP parameters)`
* Failure: 
    * `Bobcat_stuck`: `P_f(explicit subset of the PLP parameters)`
    * `Bobcat_turned_over`: `P_f(explicit subset of the PLP parameters)`

## Running Time
* Given Success: `rt_s(explicit subset of the PLP parameters)`
* Given Failure: `rt_f(explicit subset of the PLP parameters)`