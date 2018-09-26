# Non-negative; calibrated to takeoff altitude; auto-calib if falls below takeoff height; inaccurate near ground
float32 height_m

float32 speed_northing_mps
float32 speed_easting_mps
float32 speed_horizontal_mps
float32 speed_vertical_mps

float32 flight_time_sec

bool imu_state
bool pressure_state
bool down_visual_state
bool power_state
bool battery_state
bool gravity_state
bool wind_state

uint8 imu_calibration_state
uint8 battery_percentage
float32 drone_fly_time_left_sec
float32 drone_battery_left_sec

bool is_flying
bool is_on_ground
# is_em_open True in flight, False when landed
bool is_em_open
bool is_drone_hover
bool is_outage_recording
bool is_battery_low
bool is_battery_lower
bool is_factory_mode

# flymode=1: landed; =6: flying
uint8 fly_mode
float32 throw_takeoff_timer_sec
uint8 camera_state

uint8 electrical_machinery_state

bool front_in
bool front_out
bool front_lsc

float32 temperature_height_m

float32 cmd_roll_ratio
float32 cmd_pitch_ratio
float32 cmd_yaw_ratio
float32 cmd_vspeed_ratio
bool cmd_fast_mode