# CityEnviron + PX4 WSL2 Startup Sequence

## settings.json (C:\Users\14037\Documents\AirSim\settings.json)
- LocalHostIp: "0.0.0.0"  (CityEnviron listens on all Windows interfaces)
- ControlIp: "remote"     (UDP control resolves automatically)
- LockStep: false

## WSL2 PX4
export PX4_SIM_HOST_ADDR=172.24.48.1  (Windows gateway IP)
cd ~/PX4-Autopilot && make px4_sitl_default none_iris

## MAVROS (after CityEnviron connects)
# Start new MAVLink instance in PX4 console:
mavlink start -u 14600 -r 4000000 -m onboard -o 14601
# Then launch MAVROS:
roslaunch mavros px4.launch fcu_url:="udp://:14601@localhost:14600"

## Key IPs
- WSL2 IP: 172.24.56.104
- Windows gateway: 172.24.48.1
