# Precipitation Quantity
float32 absolute # UMB Channel 620, Absolute, 0-100000 [mm]
bool absolute_valid
float32 differential # UMB Channel 625, Differential, 0-100000 [mm]
bool differential_valid

# Precipitation Type
# 0 = No precipitation
# 40 = unspecified precipitation
# 60 = Liquid precipitation, e.g. rain
# 67 = freezing rain
# 69 = sleet
# 70 = Solid precipitation, e.g. snow
# 90 = hail
uint8 type  # UMB Channel 700, [1]
bool type_valid

# WMO Code
# WMO Synop Code 4680 wawa (Present Weather, automated station)
uint8 code # UMB Channel 780, [1]
bool code_valid

# Precipitation Intensity
float32 intensity_h   # UMB Channel 820, 0.00 - 200.00 [mm/h]
bool intensity_h_valid
float32 intensity_min # UMB Channel 825, 0.00 - 3.3333 [mm/min] 
bool intensity_min_valid