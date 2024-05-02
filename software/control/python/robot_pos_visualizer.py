import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import random
import time

from maru import osx001Driver
from serial.tools import list_ports

# The acquisition of information from robots may be delayed because the amount of communication increases as the number of robots increases. There is a problem with the smooth acquisition of location information for robots with more than four units in a Windows environment. No problem on Mac and Ubuntu (probably an OS-derived Thread implementation issue).

# maximum number of robots
robt_num = 12

# Enumerate the port names of the connected cradle
# ports = ["COM3", "COM2"]
ports = ["/dev/cu.usbserial-AR0KBALD", "/dev/cu.usbserial-AR0KBALC"]

# Initial robot positions.
positions = np.full((robt_num, 2), [450, 300], dtype=np.float64)

drivers = []
for i in range(len(ports)):
    driver = osx001Driver(ports[i])
    drivers.append(driver)


fig, ax = plt.subplots()
sc = ax.scatter(positions[:, 0], positions[:, 1])

ax.set_xlim(-450, 450)
ax.set_ylim(-300, 300)

def update(frame):
    sc.set_offsets(positions)
    return sc,

def fetch_data():

    while True:
        for i in range(len(drivers)):
            driver = drivers[i]

            data_dict = driver.checkStatusPacket()

            if data_dict is not None:
                for id in data_dict:
                    positions[id-1, 0] = data_dict[id][0]
                    positions[id-1, 1] = data_dict[id][1]

        # time.sleep(0.1)  # delay

thread = threading.Thread(target=fetch_data)
thread.daemon = True
thread.start()

ani = FuncAnimation(fig, update, interval=100)  # Updated every 100 milliseconds

plt.show()
