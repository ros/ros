# A 2-D grid map, in which each cell represents the probability of
# occupancy.  Occupancy values are integers in the range [0,100], or -1 
# for unknown.

# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
Pose2DFloat32 origin
# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
byte[] data
