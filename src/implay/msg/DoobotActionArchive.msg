# Action messages:
# 0 - do nothing
# 1 - tilt left
# 2 - tilt right
# 3 - fire solenoid
# 4 - start servo

Header header

uint32 action      # Action to take
uint32 action_src  # 1 = argmax, 2 = dist
uint32 fseq        # Frame
