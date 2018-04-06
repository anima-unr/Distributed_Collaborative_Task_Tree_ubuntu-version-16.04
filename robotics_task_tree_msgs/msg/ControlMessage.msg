# Control message type
# 0: state and control
# 1: state only 

NodeBitmask sender
int32 type
float32 activation_level
float32 activation_potential
bool done
bool active
NodeBitmask highest
int32 parent_type
