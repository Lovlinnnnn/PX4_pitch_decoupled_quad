# Measured common rotor tilt about body +Y.
uint64 timestamp # PX4 receipt time, microseconds since system start
uint64 timestamp_sample # Receipt time minus the reported sample age; excludes link transit time
float32 angle_rad # Actual angle, or last healthy measurement when invalid; NaN before first valid sample
uint32 sequence # Companion sequence with uint32 wrap
bool valid # Current packet contains a fresh, healthy measurement
bool fault # Companion reports a servo feedback fault

# TOPICS tilting_servo_status
