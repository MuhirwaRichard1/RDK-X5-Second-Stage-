# Fused free-space sectors, spanning the robot's field of view.
#
# Sector i covers bearings [angle_min + i*w, angle_min + (i+1)*w] where
# w = (angle_max - angle_min) / len(status). Bearings follow REP-103:
# radians, 0 = straight ahead, counter-clockwise positive (left = +).
#
# UNKNOWN means "no camera data for this direction" and MUST be treated
# as BLOCKED by consumers (fail-safe, see PROPOSAL).

uint8 UNKNOWN=0
uint8 FREE=1
uint8 BLOCKED=2

std_msgs/Header header
float32 angle_min
float32 angle_max
uint8[] status          # one of the constants above, per sector
float32[] free_fraction # 0..1 drivable evidence per sector (0 when UNKNOWN)
