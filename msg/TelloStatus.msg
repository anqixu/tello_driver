int16 height_m

int16 speed_front_mps
int16 speed_lateral_mps
int16 speed_planar_mps
int16 speed_vertical_mps

uint16 flight_time_sec

bool imu_state
bool pressure_state
bool down_visual_state
bool power_state
bool battery_state
bool gravity_state
bool wind_state

uint8 imu_calibration_state
uint8 battery_percentage
uint16 drone_fly_time_left_sec
uint16 drone_battery_left_sec

bool is_flying
bool is_on_ground
bool is_em_open
bool is_drone_hover
bool is_outage_recording
bool is_battery_low
bool is_battery_lower
bool is_factory_mode

uint8 fly_mode
uint8 throw_takeoff_timer_sec
uint8 camera_state

# TODO: suspect this is related to overheating
uint8 electrical_machinery_state

bool front_in
bool front_out
bool front_lsc

uint8 temperature_height_m