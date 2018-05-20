from rplidar import RPLidar
lidar = RPLidar('COM3')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

for i, scan in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(scan)))
    print(scan)
    for j, meas in enumerate(scan):
        print(meas)

    break

print("Test round:")
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
