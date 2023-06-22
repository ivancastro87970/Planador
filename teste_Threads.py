import time
import numpy as np
from ahrs.filters import Madgwick
from ahrs.common import Quaternion
import qwiic_icm20948
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ahrs.common import quaternion as quat
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
import math
import logging
import threading

# Teste do código implementado usando Threads

sampling_rate = 100.0
madgwick_filter = Madgwick()
samples = 0
quaternions = [1.0, 0, 0, 0]
gyro_x_raw = 0
gyro_y_raw = 0
gyro_z_raw = 0
gyro_x_offset = 0
gyro_y_offset = 0
gyro_z_offset = 0
a = 2
q_previous = np.array([1.0, 0, 0, 0])
qT = np.array([1.0, 0, 0, 0])

def _getData(name):
    global samples
    global sampling_rate
    global quaternions
    global gyro_x_raw
    global gyro_y_raw
    global gyro_z_raw
    global gyro_x_offset
    global gyro_y_offset
    global gyro_z_offset
    global q_previous
    global qT
    global a

    while (a > 0):

        imu.getAgmt()

        if (samples < 128):
            gyro_x_offset += imu.gxRaw
            gyro_y_offset += imu.gyRaw
            gyro_z_offset += imu.gzRaw
            samples += 1

        elif (samples == 128):
            gyro_x_offset /= 64
            gyro_y_offset /= 64
            gyro_z_offset /= 64
            samples += 1

        else:
            gyro_x_raw -= gyro_x_offset
            gyro_y_raw -= gyro_y_offset
            gyro_z_raw -= gyro_z_offset

        # print(gyro_x_offset, gyro_y_offset, gyro_z_offset)

        acc = imu.axRaw, imu.ayRaw, imu.azRaw
        gyro = imu.gxRaw, imu.gyRaw, imu.gzRaw
        # gyro = gyro_x_raw,gyro_y_raw, gyro_z_raw
        mag = imu.mxRaw, imu.myRaw, imu.mzRaw
        # print(gyro)

        acc_SI = [x * 9.81 for x in acc]
        gyro_SI = [x * 0.0174533 for x in gyro]
        # gyro_SI = gyro

        # print(gyro)
        # print(mag)
        # q = np.array([1., -1., -1., -1.])
        # q= q_previous
        # q = np.array[q]
        q = madgwick_filter.updateMARG(q_previous, np.array(gyro_SI), np.array(acc_SI), np.array(mag))
        q_previous = q

        quaternions.append(Quaternion(q))

        # print ("Acelarómetro : X=",imu.axRaw, "Y=", imu.ayRaw, "Z=", imu.azRaw,"Magnetometro : X=", imu.mxRaw, "Y=",imu.myRaw, "Z=", imu.mzRaw)

        # print(q)
        qT = q
        time.sleep(0.5)

    # print ("Giroscópio : X=",imu.gxRaw, "Y=", imu.gyRaw, "Z=", imu.gzRaw)
    # print ("Magnetometro : X=", imu.mxRaw, "Y=",imu.myRaw, "Z=", imu.mzRaw)

def plot_orientacao():
    # q= _getData()
    global qT
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    faces = []

    vertices = np.array([[-1, -1, -1],
                         [1, -1, -1],
                         [1, 1, -1],
                         [-1, 1, -1],
                         [-1, -1, 1],
                         [1, -1, 1],
                         [1, 1, 1],
                         [-1, 1, 1]])

    def update(frame):
        global qT

        # q1= _getData()
        # print(qT)
        q0 = qT[0]
        q1 = qT[1]
        q2 = qT[2]
        q3 = qT[3]
        rot_matrix = np.array([[1 - 2 * q2 ** 2 - 2 * q3 ** 2, 2 * q1 * q2 - 2 * q3 * q0, 2 * q1 * q3 + 2 * q2 * q0],
                               [2 * q1 * q2 + 2 * q3 * q0, 1 - 2 * q1 ** 2 - 2 * q3 ** 2, 2 * q2 * q3 - 2 * q1 * q0],
                               [2 * q1 * q3 - 2 * q2 * q0, 2 * q2 * q3 + 2 * q1 * q0, 1 - 2 * q1 ** 2 - 2 * q2 ** 2]])

        rotated_vertices = np.dot(vertices, rot_matrix.T)

        for face in faces:
            face.set_verts([])

        # Desenha as faces do cubo com cores diferentes
        colors = ['red', 'green', 'blue', 'yellow', 'orange', 'purple']
        for i, color in enumerate(colors):
            face = ax.add_collection3d(Poly3DCollection([rotated_vertices[[i for i in range(4)]]], alpha=0.5))
            face.set_facecolor(color)
            faces.append(face)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])

        return faces

    def init():
        return update(1)

    ani = FuncAnimation(fig, update, init_func=init, frames=100, interval=2)
    plt.show()  # .draw cria mts janelas
    plt.pause(1 / sampling_rate)
    return

# def thread_function(name):
#   global qT
#  while(a>0):
# qT= _getData()
#     time.sleep(10)
# print(1)

imu = qwiic_icm20948.QwiicIcm20948()

x = threading.Thread(target=_getData, args=(1,))
x.start()

if imu.connected:
    print("IMU conectado!")
    imu.begin()
else:
    print("IMU não conectado. Verifique a conexão e tente novamente.")
    exit()

# while(a>0)

# qT = _getData()
plot_orientacao()