from rplidar import RPLidar
from itertools import groupby
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import ptimer
import cv2
import math

PORT_NAME = 'COM3'
AMOUNT_OF_SCANS = 10
MAX_DISTANCE = 6000
IMIN = 0
IMAX = 50
EVEN_OR_UNEVEN = True


def compare_image(figure):
    figure.savefig("figure2.png")
    image1 = cv2.imread("figure1.png")
    image2 = cv2.imread("figure2.png")
    difference = cv2.subtract(image1, image2)
    cv2.imwrite("result.png", difference)


def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intensity = np.array([meas[0] for meas in scan])
    line.set_array(intensity)
    return line


def plot_image(iterator):
    fig = plt.figure()
    axis = plt.subplot(111, projection='polar')
    line = axis.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX], cmap=plt.cm.Greys_r, lw=0)
    axis.set_rmax(MAX_DISTANCE)
    axis.grid(True)

    ani = animation.FuncAnimation(fig, update_line, fargs=(iterator, line), interval=50, save_count=1)
    # ani.save('meetingen.mp4') # Getting errors need to fix.
    plt.show()
    fig.savefig("figure.png")
    ptimer.repeat(compare_image(fig), 2)
    plt.close(fig)


def add_scans(iterator, num):
    data_list = []      # Adds the data from multiple scans and sorts them based on angle.
    rounded_list = []   # Data list with values rounded to intergers and no quality values.
    grouped_list = []   # Rounded list but with only one value per angle (average of all values on that angle)

    for i in range(num):
        data_list += scan_lidar(iterator)
    data_list.sort(key=lambda tup: tup[1])

    for point in data_list:
        rounded_list.append((round(point[1]), round(point[2])))

    groups = groupby(rounded_list, lambda tup: tup[0])
    for group_angle, group_tuples in groups:
        values = [t[1] for t in group_tuples]
        grouped_list.append((group_angle, round(sum(values) / len(values))))

    return grouped_list


def scan_lidar(iterator):
    scan = next(iterator)
    return scan


def run():
    lidar = RPLidar(PORT_NAME)
    lidar.stop()
    iterator = lidar.iter_scans()
    print("A single scan:")
    print(scan_lidar(iterator))
    print("added multiple scans:")
    print(add_scans(iterator, AMOUNT_OF_SCANS))
    print("Give objects")
    sort_into_objects(scan_lidar(iterator))
    plot_image(iterator)
    lidar.stop()
    lidar.disconnect()


def measure():
    lidar = RPLidar(PORT_NAME)
    lidar.stop()
    iterator = lidar.iter_scans()
    values = add_scans(iterator, AMOUNT_OF_SCANS)
    lidar.stop()
    lidar.disconnect()
    return values


def measure_angle(angle):  # Return distance measured on/near given angle.
    lidar = RPLidar(PORT_NAME)
    lidar.stop()
    iterator = lidar.iter_scans()
    list = scan_lidar(iterator)
    smallest_difference = 360
    distance = 0
    for point in list:
        if abs(point[1] - angle) < smallest_difference:
            distance = point[2]
    return distance


def sort_into_objects(scan):
    distance = 2
    angle = 1
    current_list = []
    previous_angle = 0  # Previous angle in the list.
    previous_distance = 0  # Previous distance in the list.
    angle_resolution = 2  # based on datasheet.
    dist_resolution_border = 1500  # See RPLIDAR A1 Datasheet specifications.

    current_object = []  # A list that takes multiple datapoints and turns them into a single object.
    smallest_distance = 0  # Smallest distance to the object. ( This is for the smallest distance mode.)

    final_list = []  # A list with all the objects made with the current_object list.
    for j, meas in enumerate(scan):
        current_list.append(meas)

    for i in current_list:
        if i == current_list[0]:    # add first measurement to the list
            previous_distance = i[distance]
            previous_angle = i[angle]
        if i == current_list[len(current_list) - 1]:
            current_object.append(i)
            for k in current_object:
                if k == current_object[0]:
                    smallest_distance = k[distance]
                if k[2] < smallest_distance:
                    smallest_distance = k[distance]

            start_point = current_object[0]
            end_point = current_object[len(current_object) - 1]

            dif_angle = abs(start_point[angle] - end_point[angle])
            average_angle = (start_point[angle] + end_point[angle]) / 2
            final_list.append((smallest_distance, average_angle,
                               cosine_law_sas(start_point[distance], dif_angle, end_point[distance])))

        distance_resolution = 50  # Based on datasheet for Model A1M8
        if i[distance] > dist_resolution_border:  # See RPLIDAR A1 Datasheet specifications.
            distance_resolution = i[distance] / 100
        if not current_object:
            current_object.append(i)
        if (abs(i[angle] - previous_angle) <= angle_resolution) and \
                (abs(i[distance] - previous_distance) <= distance_resolution):
            # If the difference between angles or distance is too large make a new obstacle.
            current_object.append(i)  # Add current measurement to new obstacle.
        else:
            for j in current_object:
                if j == current_object[0]:  # Set smallest distance to the first distance in the object.
                    smallest_distance = j[distance]

                if j[2] < smallest_distance:  # If any distance in the object is smaller that becomes the smallest.
                    smallest_distance = j[distance]

            start_point = (current_object[0])  # Start angle is the first angle measured.
            end_point = current_object[len(current_object) - 1]  # End angle is the last angle measured.
            average_angle = (start_point[angle] + end_point[angle]) / 2
            dif_angle = abs(start_point[angle] - end_point[angle])
            final_list.append((smallest_distance, average_angle,
                               cosine_law_sas(start_point[distance], dif_angle, end_point[distance])))
            # Add obstacle to obstacle list.
            current_object = []
        previous_angle = i[angle]
        previous_distance = i[angle]

    print(final_list)


def cosine_law_sas(distance_b, angle, distance_c):
    test = math.cos(angle)
    a_squared = math.pow(distance_b, 2) + math.pow(distance_c, 2) - 2 * distance_b * distance_c * math.cos(math.radians(angle))
    side_a = math.pow(a_squared, 0.5)
    return side_a


run()
