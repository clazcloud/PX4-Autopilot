# Burl governor state, published by FlightTaskManualAltitude

uint64 timestamp

uint8 state                 # matches GovState in FlightTaskManualAltitude
uint8 STATE_OFF = 0
uint8 STATE_HEADLESS = 1
uint8 STATE_CYCLIC = 2
uint8 STATE_SPIN_DOWN = 3

float32 yawspeed_sp         # [rad/s] ramped spin rate command
float32 yawspeed_measured   # [rad/s] measured body yaw rate
float32 ref_heading         # [rad] headless reference heading
