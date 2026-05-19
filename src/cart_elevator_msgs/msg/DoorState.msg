# Elevator door state, as estimated from D455 depth observations over a
# fixed ROI in front of the cart. Published by door_state_detector at a
# steady rate so the supervisor can event-drive on transitions.

std_msgs/Header header

uint8 UNKNOWN  = 0   # detector not yet warm or ROI invalid
uint8 CLOSED   = 1   # depth in ROI consistently shallow (door surface)
uint8 OPENING  = 2   # depth-boundary moving sideways, lateral profile changing
uint8 OPEN     = 3   # depth in ROI consistently deep (into the cab)
uint8 CLOSING  = 4   # transient, opposite of OPENING

uint8 state
float64 confidence    # 0.0..1.0 of the chosen state
float64 median_depth_m   # median depth in the ROI right now, for debug/log
