# This is the configuration file for the IBEO PLP. Feel free to change the values -
# but don't change the keys!
config = {
    "TIME_INCREMENT_TOLERANCE" : 0.01, # seconds
    "TIME_INCREMENT": 0.08, # seconds
    "FAIL_OR_COVER_THRESHOLD": 0.1,  # ratio (percent/100)
    "OBSTACLE_THRESHOLD": 0.3, # ratio (percent/100)
    "SKY_THRESHOLD": 0.8, # ratio (percent/100)
    "FAIL_OR_COVER_DISTANCE": 1, # meter - distance under which we assume IBEO is blocked
    "OBSTACLE_DISTANCE": 3, # meter - distance under which we assume close obstacle exists
    "SKY_DISTANCE": 50, # meter - distance over which we assume IBEO is looking at the sky
    "RAYS":["t1","t2","b1","b2"],  # Names of the rays in the message
    "WATCHDOG_PERIOD": 0.5 # seconds
}
