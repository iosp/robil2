# Camera Maintain PLP

1. 2015-12-21 Initial version

## About
Maintains a video feed that's useful for navigation. Implemented using histogram analysis on the camera's video signal.

## Values
### Parameters
* `videoFeed` &larr; `/TO/BE/DECIDED` The camera video feed.

### Constants
* `IDEAL_DISTRIBUTION` Ideal distribution of the pixels over the histogram bins in the video feed.
* `DEVIATION_THRESHOLD` Threshold of deviation from `IDEAL_DISTRIBUTION`, above which the PLP detects that the camera input is not useful.

### Variables
_Calculated based on parameters and constants._

* `currentHistorgram` Histogram of the current frame in the video feed

## Natural Preconditions
- Camera on

## Maintained Condition
* `abs( process(currentHistorgram) - process(IDEAL_DISTRIBUTION) ) =< DEVIATION_THRESHOLD`

## Successful Termination Condition
* (none) _module does not termiante_

## Failure Termination Condition
* No video feed.

## Success Probability
* ???
  * _Not sure what to put here... do we have any fixing actions? How can the module affect this?_

## Failure Probability
* ???
  * _see above_

## Running Time Distribution

_This module runs while the system is up_

* Given Success `TBD?`
* Given Failure
  * For each failure mode `TBD?`


## Application Context
### Resources
* Camera video feed (non-exclusive)

### Concurrency Conditions
_Later, we will replace this with "health" PLP for the entire system_

* `CAMERA_OK`

### Concurrent Modules
* (none)

## Side Effects
* (none)

## Robil Integration

### Trigger
* System on

### Abort
* (none)

### Output
* Alerts go to the `plp/messages` ROS topic.
