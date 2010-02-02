Header header
# Is the planner actively going to a goal?
byte active
# Did the planner find a valid path?
byte valid
# Have we arrived at the goal?
byte done
# Current location (m,m,rad)
Pose2DFloat32 pos
# Goal location (m,m,rad)
Pose2DFloat32 goal
# Current waypoint location (m,m,rad)
Pose2DFloat32 waypoint
# Current list of waypoints in the plan
Pose2DFloat32[] waypoints
# Current waypoint index. May be negative if there's no plan, or if
# the plan is done
int32 waypoint_idx
