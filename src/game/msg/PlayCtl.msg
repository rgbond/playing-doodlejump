# Play control dialog
#
# train (passed to tf_play)
# 1: start training
# 
# tf_state msgs (passed back to archive):
# 0 - startup
# 1 - online
# 2 - training
#
# msg_src
# 0 - archive
# 1 - tf_play

Header header

int32 train   
int32 tf_state
int32 msg_src
