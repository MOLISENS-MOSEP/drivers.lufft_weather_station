float32 speed                           # UMB Channel 400, 0 - 75.0 [m/s]
bool speed_valid
float32 speed_min                       # UMB Channel 420, 0 - 75.0 [m/s]
bool speed_min_valid
float32 speed_max                       # UMB Channel 440, 0 - 75.0 [m/s]
bool speed_max_valid
float32 speed_avg                       # UMB Channel 460, 0 - 75.0 [m/s]
bool speed_avg_valid
float32 speed_vector_avg                # UMB Channel 480, 0 - 75.0 [m/s]
bool speed_vector_avg_valid

float32 speed_fast                      # UMB Channel 401, 0 - 75.0 [m/s]
bool speed_fast_valid

float32 speed_std        # UMB Channel 403, 0 - 75.0 [m/s]
bool speed_std_valid

float32 direction                       # UMB Channel 500, 0 - 359.9 [°]
bool direction_valid
float32 direction_min                   # UMB Channel 520, 0 - 359.9 [°]
bool direction_min_valid
float32 direction_max                   # UMB Channel 540, 0 - 359.9 [°]
bool direction_max_valid
float32 direction_vector_avg            # UMB Channel 580, 0 - 359.9 [°]
bool direction_vector_avg_valid

float32 direction_fast                  # UMB Channel 501, 0 - 359.9 [°]
bool direction_fast_valid
float32 direction_corrected             # UMB Channel 502, 0 - 359.9 [°]
bool direction_corrected_valid
float32 direction_std                   # UMB Channel 503, 0 - 359.9 [°]
bool direction_std_valid
float32 value_quality                   # UMB Channel 805, 0 - 100 [%]
bool value_quality_valid
float32 value_quality_fast              # UMB Channel 806, 0 - 100 [%]
bool value_quality_fast_valid