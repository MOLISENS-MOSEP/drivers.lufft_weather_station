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


    620: "precipitation.absolute",
    625: "precipitation.differential",
    700: "precipitation.type",
    780: "precipitation.code",
    820: "precipitation.intensity_hour",
    825: "precipitation.intensity_minute",

    

}



