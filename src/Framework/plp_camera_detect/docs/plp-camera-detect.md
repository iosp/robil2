# Camera Detect PLP

1. 2015-12-21 Initial version

## About
Detects when the camera input is not useful for navigation. Implemented using histogram analysis on the camera's video signal.

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

## Detection Goal
* `abs( process(currentHistorgram) - process(IDEAL_DISTRIBUTION) ) > DEVIATION_THRESHOLD`

## Probability of Successful Detection Given Condition
* `TBD`
  * The goal to be detected is over digital signal, so we can claim 100%. But what is the probability, given that the camera input is not useful, that the video signal still has an OK distribution?


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
