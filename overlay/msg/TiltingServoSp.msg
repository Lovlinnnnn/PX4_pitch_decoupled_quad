# Command for the tilting servo before mechanical calibration.
uint64 timestamp # Microseconds since system start
uint8 NUM_SERVO_OUTPUTS = 4
float32[4] angle # Radians about body +Y; single tilt uses angle[0]

# TOPICS tilting_servo_setpoint
