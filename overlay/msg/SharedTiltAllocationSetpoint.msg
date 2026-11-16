# Common-tilt wrench input to the native control allocator.
uint64 timestamp # Time of this request, microseconds since system start
uint64 timestamp_sample # Time of the tilt sample used in the projection
float32 tilt_rad # Measured common tilt in FRD, positive towards the nose
float32 collective # Fraction of the available total thrust above armed idle
uint8 mode
uint8 INHIBIT = 0
uint8 HOLD = 1
uint8 ACTIVE = 2
