from rplidar import RPLidar
from itertools import groupby
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import ptimer
import cv2

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
    print(add_scans(iterator, AMOUNT_OF_SCANS))
    plot_image(iterator)
    lidar.stop()
    lidar.disconnect()


run()
