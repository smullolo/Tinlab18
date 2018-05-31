from rplidar import RPLidar

lidar = RPLidar('COM3')  # Connect to the lidar on COM3

info = lidar.get_info()  # Get the model info from the lidar
print(info)  # Print the model info

health = lidar.get_health()  # Get the health of the lidar. If the health is good the software hasn't found any issues.
print(health)  # Print the health of the lidar.

previous_angle = 0  # Previous angle in the list.
previous_distance = 0  # Previous distance in the list.
angle_resolution = 1  # based on datasheet.

current_object = []  # A list that takes multiple datapoints and turns them into a single object.
start_angle = 0  # Start angle of the new object.
end_angle = 0  # End angle of the new object.
start_distance = 0  # Start distance of the new object.
smallest_distance = 0  # Smallest distance to the object. ( This is for the smallest distance mode.)

final_list = []  # A list with all the objects made with the current_object list.

for i, scan in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(scan)))  # Check how many measurements we've made.
    print(scan)  # Print all the measured results.
    current_list = []
    for j, meas in enumerate(scan):
        current_list.append(meas)  # Add the measured results to a new list.
    for i in current_list:
        if i == current_list[0]:  # Set previous distance and previous angle to first versions measured.
            previous_distance = i[2]
            previous_angle = i[1]
        if i == current_list[len(current_list) - 1]:
            current_object.append(i)  # If it's the last object in the list add it to the last object.
            # (if the previous object was already closed this makes a new object.
            for j in current_object:
                if j == current_object[0]:  # Set smallest distance to the first distance in the object.
                    smallest_distance = j[2]

                if j[2] < smallest_distance:  # If any distance in the object is smaller that becomes the smallest.
                    smallest_distance = j[2]

            start_angle = (current_object[0])[1]  # Start angle is the first angle measured.
            end_angle = current_object[len(current_object) - 1][1]  # End angle is the last angle measured.
            final_list.append((smallest_distance, start_angle, end_angle))  # Add obstacle to obstacle list.
        distance_resolution = 0.5  # Based on datasheet for Model A1M8
        if i[2] > 1500:  # See RPLIDAR A1 Datasheet specifications.
            distance_resolution = i[2] / 100
        if (i[1] - previous_angle <= 1) and (i[2] - previous_distance <= distance_resolution):
            # If the difference between angles or distance is too large make a new obstacle.
            current_object.append(i)  # Add current measurement to new obstacle.
        else:
            for j in current_object:
                if j == current_object[0]:  # Set smallest distance to the first distance in the object.
                    smallest_distance = j[2]

                if j[2] < smallest_distance:  # If any distance in the object is smaller that becomes the smallest.
                    smallest_distance = j[2]

            start_angle = (current_object[0])[1]  # Start angle is the first angle measured.
            end_angle = current_object[len(current_object) - 1][1]  # End angle is the last angle measured.
            final_list.append((smallest_distance, start_angle, end_angle))  # Add obstacle to obstacle list.
            print("Nieuw object: ")
            print(current_object)
            current_object = [i]  # Make a new obstacle if previous obstacle is done.
            # min max berekene
            # avg angle
            # append final list
        # clear obj list
        previous_angle = i[1]  # Set previous angle to current angle
        previous_distance = i[2]  # Set previous distance to current distance.
        # print(i)
    print(final_list)  # Print list of obstacles.

    break

lidar.stop()  # Stop the lidar.
lidar.stop_motor()  # Stop the motor.
lidar.disconnect()  # Disconnect the lidar.
