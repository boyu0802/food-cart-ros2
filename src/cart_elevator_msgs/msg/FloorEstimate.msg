# Estimated current elevator floor with confidence + source tag.
# Emitted by each floor_v*.py detector and the fusion node.
# All four sources publish this same type so they're interchangeable.

std_msgs/Header header

int32 floor               # estimated floor number (building convention; 1 = ground)
float64 confidence        # 0.0..1.0. AprilTag ~0.95, time-based ~0.6, accel ~0.4.

uint8 SOURCE_UNKNOWN = 0
uint8 SOURCE_APRILTAG = 1
uint8 SOURCE_TIME = 2
uint8 SOURCE_ACCEL = 3
uint8 SOURCE_FUSION = 4
uint8 source

bool moving               # true if elevator currently in motion (best guess)
int8 direction            # -1 = down, 0 = stationary / unknown, +1 = up
