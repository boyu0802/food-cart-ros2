# Estimated current travel direction of the elevator, from the animated
# arrow on the in-car display (NOT from IMU — that's covered separately).
#
# Emitted by each direction_v*.py detector. Same shape as FloorEstimate
# so the two stacks feel symmetric.

std_msgs/Header header

int8 DIRECTION_UNKNOWN = 0
int8 DIRECTION_UP = 1
int8 DIRECTION_DOWN = -1
int8 DIRECTION_IDLE = 2     # display visible, no animation
int8 direction

float64 confidence          # 0.0..1.0

uint8 SOURCE_UNKNOWN = 0
uint8 SOURCE_DIGIT_TRACK = 1   # direction_v1_digit -- primary
uint8 SOURCE_OPTICAL_FLOW = 2  # direction_v2_flow -- arrow-based fallback (PLACEHOLDER)
uint8 SOURCE_FUSION = 3
uint8 source
