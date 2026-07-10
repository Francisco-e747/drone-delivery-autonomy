# PX4 SITL parameters for CityEnviron + AirSim
# Apply these in PX4 console or add to rcS

# EKF2 GPS settings
param set EKF2_GPS_DELAY 110
param set EKF2_GPS_CHECK 0
param set EKF2_HGT_MODE 0

# Enable external vision position + yaw (for xy_valid fix)
param set EKF2_AID_MASK 25
param set EKF2_EV_DELAY 0
param set EKF2_MAG_TYPE 5

# IMU
param set IMU_INTEG_RATE 250
