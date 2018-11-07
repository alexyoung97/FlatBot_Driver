#--------------------------------------
#
# FlatBot ConnectorStatus message
#
# represents the current state of an
# active androgynous connector port 
#
#--------------------------------------

# id number of this connector
int16 id

# current status of this connector
int16 status

# valid status values
int16 UNKNOWN=-1
int16 OPEN=0
int16 LOCKED=1
int16 CLOSED_FAILED=2
int16 OPEN_CONNECTED=3

int16 CLOSING=4
int16 OPENING_FROM_LOCKED=5
int16 OPENING_FROM_FAILED=6

int16 STUCK_IN_TRANSISION_STATE=7
int16 BOOTED_IN_TRANSITION_STATE=8

# current error status of this connector
int16 error

# valid errors
int16 ERROR_OKAY=0
int16 ERROR_OPEN_MOTION_FAILED=1
int16 ERROR_CLOSE_MOTION_FAILED=2
int16 ERROR_TRANSITION_STATE=3

# connection switch state, true if a connection is detected
bool switch_closed

# real value between 0 - 1 where 0 is fully open and 1 is fully closed.
float32 latch_position
