bool in_autonomous_mode
bool watchdog_tripped
float32 voltage

int8 robot_power_state
int8 POWER_GOOD = 0
int8 POWER_LOW = 1
int8 POWER_EMERGENCY = 2
int8 ESTOP_ACTIVE = 3

int8 auton_state
int8 MOVE_FWD = 0
int8 MOVE_BWD = 1
int8 MOVE_LEFT = 2
int8 MOVE_RIGHT = 3
int8 WHICH_WAY_SLCT = 4
int8 DELAY_FWD_HEAD = 5
int8 DELAY_BWD_HEAD = 6
int8 ERROR = 7
