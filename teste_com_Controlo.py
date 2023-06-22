import time
import pi_servo_hat
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
a = 1
q_previous = [1.0, 0, 0, 0]
quaternio_buffer = []

kp = 10
ki = 0.1
integral = 0.0
erro = 0.0

kpRoll = 10
kiRoll = 0.1
integralRoll = 0.0
erroRoll = 0.0

media_atual = [0, 0, 0, 0]

class SlidingAverageQuaternion:
    def __init__(self):
        self.quaternions = []
        self.avg_q = [1.0, 0, 0, 0]

    def add_quaternion(self, q11, q22, q33, q44):
        qT = [q11, q22, q33, q44]
        self.quaternions.append(qT)
        n = len(self.quaternions)

        if (n > 7):
            self.quaternions.pop(0)
            n -= 1

        # Atualiza a média deslizante
        for i in range(4):
            self.avg_q[i] = (self.avg_q[i] * (n - 1) + qT[i]) / n

        # self.quaternions.pop(0)

    def get_average(self):
        return self.avg_q

def _controloPitch(x):
    global erro
    global integral

    erro = 20 - x
    integral = integral + ki * erro

    return kp * erro + ki * integral

def _controloRoll(x):
    global erroRoll
    global integralRoll

    erroRoll = 0 - x
    integralRoll = integral + kiRoll * erroRoll

    return kpRoll * erroRoll + kiRoll * integralRoll

def _getPitch(q):
    rot_matrix = np.array(
        [[1 - 2 * q[2] ** 2 - 2 * q[3] ** 2, 2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[1] * q[3] + 2 * q[0] * q[2]],
         [2 * q[1] * q[2] + 2 * q[0] * q[3], 1 - 2 * q[1] ** 2 - 2 * q[3] ** 2, 2 * q[2] * q[3] - 2 * q[0] * q[1]],
         [2 * q[1] * q[3] - 2 * q[0] * q[2], 2 * q[2] * q[3] + 2 * q[0] * q[1], 1 - 2 * q[1] ** 2 - 2 * q[2] ** 2]])

    pitch = np.arctan2(-rot_matrix[2, 0], np.sqrt(rot_matrix[2, 1] ** 2 + rot_matrix[2, 2] ** 2))

    return np.degrees(pitch)

def _getRoll(q):
    rot_matrix = np.array(
        [[1 - 2 * q[2] ** 2 - 2 * q[3] ** 2, 2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[1] * q[3] + 2 * q[0] * q[2]],
         [2 * q[1] * q[2] + 2 * q[0] * q[3], 1 - 2 * q[1] ** 2 - 2 * q[3] ** 2, 2 * q[2] * q[3] - 2 * q[0] * q[1]],
         [2 * q[1] * q[3] - 2 * q[0] * q[2], 2 * q[2] * q[3] + 2 * q[0] * q[1], 1 - 2 * q[1] ** 2 - 2 * q[2] ** 2]])

    roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])

    return np.degrees(roll)

def _getData():
    global samples
    global sampling_rate
    global quaternions
    global gyro_x_raw
    global gyro_y_raw
    global gyro_z_raw
    global gyro_x_offset
    global gyro_y_offset
    global gyro_z_offset
    global a
    global q_previous

    imu.getAgmt()

    acc = imu.axRaw, imu.ayRaw, imu.azRaw
    gyro = imu.gxRaw, imu.gyRaw, imu.gzRaw
    mag = imu.mxRaw, imu.myRaw, imu.mzRaw

    acc_SI = [x * 9.81 for x in acc]
    gyro_SI = [x * 0.0174533 for x in gyro]
    # gyro_SI = gyro

    q = madgwick_filter.updateMARG(q_previous, np.array(gyro_SI), np.array(acc_SI), np.array(mag))
    sliding_avg.add_quaternion(q[0], q[1], q[2], q[3])

    q1 = sliding_avg.get_average()
    # print(sliding_avg.get_average())

    q[0] = q1[0]
    q[1] = q1[1]
    q[2] = q1[2]
    q[3] = q1[3]

    pitch = _getPitch(q)
    roll = _getRoll(q)

    print("Pitch=", pitch)
    print("Roll=", roll)

    pitch = _controloPitch(pitch)
    roll = _controloRoll(roll)

    print("Pitch=", pitch)
    print("Roll=", roll)

    quaternions.append(Quaternion(q))
    # q = q1
    # print(q)
    q_previous = q
    print("\\\\\\")

    return q

def plot_orientacao():
    q = _getData()
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
        q = _getData()
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]
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

imu = qwiic_icm20948.QwiicIcm20948()
sliding_avg = SlidingAverageQuaternion()

if imu.connected:
    print("IMU conectado!")
    imu.begin()
else:
    print("IMU não conectado. Verifique a conexão e tente novamente.")
    exit()

while (a > 0):
    plot_orientacao()