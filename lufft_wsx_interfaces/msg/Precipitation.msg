# Precipitation Quantity
float32 absolute                        # UMB Channel 620, Absolute, 0-100000 [mm]
bool absolute_valid
float32 differential                    # UMB Channel 625, Differential, 0-100000 [mm]
bool differential_valid

# Precipitation Type
# 0 = No precipitation
# 40 = unspecified precipitation
# 60 = Liquid precipitation, e.g. rain
# 67 = freezing rain
# 69 = sleet
# 70 = Solid precipitation, e.g. snow
# 90 = hail
uint8 type                              # UMB Channel 700, [1]
bool type_valid

# WMO Code
# WMO Synop Code 4680 wawa (Present Weather, automated station)
uint8 code                              # UMB Channel 780, [1]
bool code_valid

# Precipitation Intensity
float32 intensity_hour                  # UMB Channel 820, 0.00 - 200.00 [mm/h]
bool intensity_hour_valid
float32 intensity_minute                # UMB Channel 825, 0.00 - 3.3333 [mm/min] 
bool intensity_minute_valid

# Drop Diameter Distribution
uint32 total_precipitation_particles # UMB Channel 4600,
bool total_precipitation_particles_valid
uint32 total_drops                      # UMB Channel 4601,
bool total_drops_valid
uint16 drizzle_particles                # UMB Channel 4602,
bool drizzle_particles_valid
uint16 snow_particles                   # UMB Channel 4603,
bool snow_particles_valid
uint16 hail_particles                   # UMB Channel 4604,
bool hail_particles_valid
uint16 rain_class_0                     # UMB Channel 4620, drops < 0.5mm equiv. diameter
bool rain_class_0_valid
uint16 rain_class_1                     # UMB Channel 4621, drops 0.5 .. 1.0mm equiv. diameter
bool rain_class_1_valid
uint16 rain_class_2                     # UMB Channel 4622, drops 1.0 .. 1.5mm equiv. diameter
bool rain_class_2_valid
uint16 rain_class_3                     # UMB Channel 4623, drops 1.5 .. 2.0mm equiv. diameter
bool rain_class_3_valid
uint16 rain_class_4                     # UMB Channel 4624, drops 2.0 .. 2.5mm equiv. diameter
bool rain_class_4_valid
uint16 rain_class_5                     # UMB Channel 4625, drops 2.5 .. 3.0mm equiv. diameter
bool rain_class_5_valid
uint16 rain_class_6                     # UMB Channel 4626, drops 3.0 .. 3.5mm equiv. diameter
bool rain_class_6_valid
uint16 rain_class_7                     # UMB Channel 4627, drops 3.5 .. 4.0mm equiv. diameter
bool rain_class_7_valid
uint16 rain_class_8                     # UMB Channel 4628, drops 4.0 .. 4.5mm equiv. diameter
bool rain_class_8_valid
uint16 rain_class_9                     # UMB Channel 4629, drops 4.5 .. 5.0mm equiv. diameter
bool rain_class_9_valid
uint16 rain_class_10                    # UMB Channel 4630, drops 5.0 .. 5.5mm equiv. diameter
bool rain_class_10_valid
uint16 rain_class_11                    # UMB Channel 4631, drops > 5.5mm equiv. diameter
bool rain_class_11_valid