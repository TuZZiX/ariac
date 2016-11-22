string name
int32 id
int8 traceable
int8 location
geometry_msgs/PoseStamped pose
geometry_msgs/Vector3 linear

# location list
int8 none=0
int8 unassigned=1 # unconfirmed location
int8 cameraView=5 # before location separation
int8 conveyor=10
int8 ground=11
int8 AGV=20       # on any of AGVs
int8 AGV1=21
int8 AGV2=22
int8 bin=30       # on any of bins
int8 Bin1=31
int8 Bin2=32
int8 Bin3=33
int8 Bin4=34
int8 Bin5=35
int8 Bin6=36
int8 Bin7=37
int8 Bin8=38