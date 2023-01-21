# Channels look-up-table: The channels to use with the sensor + the channel name as used in the 
# corresponding message at ./lufft_wsx_interfaces/msg/.
CHANNELS_LUT = {
    100: "temperature.current",
    120: "temperature.minimum",
    140: "temperature.maximum",
    160: "temperature.average",

    110: "temperature.dewpoint",
    130: "temperature.dewpoint_min",
    150: "temperature.dewpoint_max",
    170: "temperature.dewpoint_avg",

    111: "temperature.wind_chill",

    114: "temperature.wet_bulb",

    112: "temperature.wind_heater",
    113: "temperature.r2s_heater",


    200: "humidity.relative",
    220: "humidity.relative_min",
    240: "humidity.relative_max",
    260: "humidity.relative_avg",

    205: "humidity.absolute",
    225: "humidity.absolute_min",
    245: "humidity.absolute_max",
    265: "humidity.absolute_avg",

    210: "humidity.mixing_ratio",
    230: "humidity.mixing_ratio_min",
    250: "humidity.mixing_ratio_max",
    270: "humidity.mixing_ratio_avg",


    215: "enthalpy.specific",
    
    
    300: "pressure.absolute",
    320: "pressure.absolute_min",
    340: "pressure.absolute_max",
    360: "pressure.absolute_avg",

    305: "pressure.relative",
    325: "pressure.relative_min",
    345: "pressure.relative_max",
    365: "pressure.relative_avg",
    

    310: "density.current",


    400: "wind.speed",
    420: "wind.speed_min",
    440: "wind.speed_max",
    460: "wind.speed_avg",
    480: "wind.speed_vector_avg",

    401: "wind.speed_fast",

    403: "wind.speed_std",

    500: "wind.direction",
    520: "wind.direction_min",
    540: "wind.direction_max",
    580: "wind.direction_vector_avg",

    501: "wind.direction_fast",
    502: "wind.direction_corrected",
    503: "wind.direction_std",
    805: "wind.value_quality",
    806: "wind.value_quality_fast",


    620: "precipitation.absolute",
    625: "precipitation.differential",
    700: "precipitation.type",
    780: "precipitation.code",
    820: "precipitation.intensity_hour",
    825: "precipitation.intensity_minute"

}



