# Dock controller state for downstream consumers (the supervisor).
# Published whenever the state machine transitions or at the control
# rate when in TRACK / ALIGNED so consumers don't have to scrape logs.

std_msgs/Header header

uint8 DISABLED = 0   # controller is parked (no target set, publishes zero)
uint8 WAIT     = 1   # target set but no detection yet
uint8 TRACK    = 2   # detection in, error above deadband, driving
uint8 ALIGNED  = 3   # error in deadband for align_frames in a row
uint8 LOST     = 4   # had a detection but lost it > lost_timeout ago

uint8 state
int32 target_tag_id
float64 e_x          # current pose error, m
float64 e_y
float64 e_yaw
