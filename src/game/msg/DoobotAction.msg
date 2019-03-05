# Action messages:
# 0 - do nothing
# 1 - tilt left
# 2 - tilt right
# 3 - fire solenoid
# 4 - start servo
# 

Header header

int32 action    # Action to take
int32 enable    # 1 = enable, 2 = disable
