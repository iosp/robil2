# IBEO PLP
This package contains the IBEO PLP. This PLP detects when an IBEO (or any `MultiLaserScan` emitting device) is producing useful data.

The node will listen to `/SENSORS/IBEO/1`, and detect when the IBEO is:

* Pointing at the sky
* Covered/Failing
* Too close to an obstacle
* Does not update frequently enough

Messages will be sent to `plp/messages`.

For a full, formal description of the PLP, please see [the PLP document](docs/plp_ibeo.html). You can change the values by editing the file `scripts/config.py`.

## Running the node
Command to run the node:

    rosrun plp_ibeo PlpIbeoRosHarness.py
