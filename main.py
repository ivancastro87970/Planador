#Imports do GPS
from __future__ import print_function
from time import sleep

import sys
import qwiic_titan_gps
from AltAzRange import AltAzimuthRange

#Imports do IMU
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
import threading 
import logging 

#Variaveis do IMU
sampling_rate = 100.0
madgwick_filter = Madgwick()
samples=0
quaternions = [1.0, 0, 0, 0]
a=1
q_previous=[1.0, 0, 0, 0]
quaternio_buffer = []

# Parametros controlo-PI
kp = 5
ki = 0.1
integral = 0.0
erro = 0.0

kpRoll = 5
kiRoll = 0.1
integralRoll = 0.0
erroRoll = 0.0

media_atual = [0,0,0,0] 

#//////////////////////////
#Variaveis do GPS
latitude =0.0
longitude =0.0
altitude = 0.0

azimuth =0.0
distance = 0.0
elevation = 0.0
#41.450745, -8.294013


# variaveis sinusoide de referencia
amplitude = 32
tamanho = 500  # Número de pontos na sinusoide
periodo = 500  # Período da sinusoide em unidades de tempo

t =0.0
sinusoide = 0.0
i = 0

#///////////////////
def _map(x):
    return int((x - (-40)) * (90 - 0) / (40 - (-40) + 0))

def _mapRoll(x):
    return int((x - (40)) * (120 - (-100)) / (-40 - 40) + (-100))
#def _map(x, in_min, in_max, out_min, out_max):
 #   return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def _motorPitch(x):
    test.move_servo_position(6, x, 90)

def _motorRoll(x):
    test1.move_servo_position(11, x)   

def _motorSeringa(x):
    test2.move_servo_position(7, x, 180)

#///////////////////
class SlidingAverageQuaternion:  # Media deslizante para os quaternioes
    def __init__(self):
        self.quaternions = []
        self.avg_q = [1.0, 0, 0, 0]
    
    def add_quaternion(self, q11, q22, q33, q44):
        qT = [q11, q22, q33, q44]
        self.quaternions.append(qT)
        n = len(self.quaternions)
        
        if (n >5):
            self.quaternions.pop(0)
            n -= 1

        # Atualiza a média deslizante
        for i in range(4):
            self.avg_q[i] = (self.avg_q[i] * (n - 1) + qT[i]) / n

        #self.quaternions.pop(0)     
    
    def get_average(self):
        return self.avg_q

# ///////////////////
class MediaDeslizante:  # Media deslizante para os valores de roll e pitch 
    def __init__(self):
        self.janela = 10
        self.valores = []
        self.media_deslizante = 0.0

    def adicionar_valor(self, valor):
        if len(self.valores) == self.janela:
            self.media_deslizante -= self.valores[0] / self.janela
            self.valores.pop(0)

        self.valores.append(valor)
        self.media_deslizante += valor / self.janela

    def media(self):
        return self.media_deslizante
#///////////////////////////////

#///////////////////////////////
def _getPitch(q):
    rot_matrix = np.array([[1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
                       [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1]],
                       [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2]])

    pitch = np.arctan2(-rot_matrix[2, 0], np.sqrt(rot_matrix[2, 1]**2 + rot_matrix[2, 2]**2))
    
    return np.degrees(pitch)

def _getRoll(q):
    rot_matrix = np.array([[1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
                       [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1]],
                       [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2]])

    roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
    
    return np.degrees(roll) 
#///////////////////////////////    

#///////////////////////////////
def _controloPitch(x):
    
    global erro
    global integral
    global t
    global sinusoide
    global amplitude
    global tamanho
    global periodo
    global i
    
    i +=1
    
    if(i==500):
        i =1
    
    erro = sinusoide[i] - x
    integral = integral + ki*erro
    #print(erro)   
    return kp*erro + ki*integral

def _controloRoll():
    
    global erroRoll
    global integralRoll
    global azimuth
    global kiRoll
    global kpRoll
    
    erroRoll = 0 - azimuth
    integralRoll = integral + kiRoll*erroRoll
    
    return kpRoll*erroRoll + kiRoll*integralRoll # Controlo PI do angulo de roll
#///////////////////////////////

#///////////////////////////////
def _getData(): # Vai buscar os valores do Imu
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
    
    q = madgwick_filter.updateMARG(np.array(q_previous), np.array(gyro_SI), np.array(acc_SI),  np.array(mag))

    sliding_avg.add_quaternion(q[0], q[1], q[2], q[3])
    
    q1 = sliding_avg.get_average()
        
    q[0] = q1[0]
    q[1] = q1[1]
    q[2] = q1[2]
    q[3] = q1[3]
    
    pitch = _getPitch(q)
    roll = _getRoll(q)
    
    #print("Pitch=",pitch)
    #print("Roll=",roll)
    
    media_deslizantePitch.adicionar_valor(pitch)
    media_deslizanteRoll.adicionar_valor(roll)

    pitch = media_deslizantePitch.media()
    roll = media_deslizanteRoll.media()
    
    #print("Pitch=",pitch)
    #print("Roll=",roll)
    
    pitch = _map(pitch)
    roll = _mapRoll(roll)

   if (sinusoide[i]<0):
        _motorSeringa(0)
       
    if(sinusoide[i]>0):
        _motorSeringa(180)
        #print(sinusoide[i])
    
    
    _motorPitch(pitch)
    _motorRoll(roll)

    quaternions.append(Quaternion(q))
    q_previous = q
   
    #print("\\\\\\")
    
    return q

#///////////////////////////////

#///////////////////////////////
def plot_orientacao(): #Faz a representação gráfica do sistema 
    q= _getData()
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
        q= _getData()
        q0=q[0]
        q1=q[1]
        q2=q[2]
        q3=q[3]
        rot_matrix = np.array([[1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q3*q0, 2*q1*q3 + 2*q2*q0],
                               [2*q1*q2 + 2*q3*q0, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q1*q0],
                               [2*q1*q3 - 2*q2*q0, 2*q2*q3 + 2*q1*q0, 1 - 2*q1**2 - 2*q2**2]])

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

    ani = FuncAnimation(fig, update,init_func=init, frames=100,interval=2)
    plt.show() # .draw cria mts janelas
    plt.pause(1 / sampling_rate)
    return

#///////////////////////////////

#///////////////////////////////
def run_example():
    global azimuth
    global distance
    global elavation 

    #print("SparkFun GPS Breakout - XA1110!")
    #qwiicGPS = qwiic_titan_gps.QwiicTitanGps()

    if qwiicGPS.connected is False:
        #print("Could not connect to to the SparkFun GPS Unit. Double check that\
         #     it's wired correctly.", file=sys.stderr)
        return

    while True:
        if qwiicGPS.get_nmea_data() is True:
            for k,v in qwiicGPS.gnss_messages.items(): 
                #print(k, ":", v)
                if(k == "Latitude"):
                    latitude = v
                if(k == "Longitude"):
                    longitude = v
                if(k == "Altitude"):
                    altitude = v

        trajetoria.target(latitude, longitude, altitude) # em relacao a ponto referencia definido pelo user 
       
        valores= trajetoria.calculate()

        azimuth = valores["azimuth"]
        elevation = valores["elevation"]
        distance = valores["distance"]

        #print(azimuth, elevation, distance)
        
#///////
        
# ATIVACAO DO IMU
imu = qwiic_icm20948.QwiicIcm20948()
sliding_avg = SlidingAverageQuaternion()
media_deslizantePitch = MediaDeslizante()
media_deslizanteRoll = MediaDeslizante()

# ATIVACAO DO MODULO GPS
qwiicGPS = qwiic_titan_gps.QwiicTitanGps()
qwiicGPS.begin()  # ativar o gps
AltAzimuthRange.default_observer(41.450745, -8.294013, 200) # meter aqui as coordenadas de destino 
trajetoria = AltAzimuthRange()

# Sinusoide de referência
t = np.linspace(0, periodo*2*np.pi, tamanho)  # Vetor de tempo
sinusoide = amplitude * np.sin((2*np.pi*t) / periodo)

# ATIVACAO DO MOTOR
test = pi_servo_hat.PiServoHat()
test1 = pi_servo_hat.PiServoHat()
test2 = pi_servo_hat.PiServoHat()

test.restart()
test1.restart()

if imu.connected:
    print("IMU conectado!")
    imu.begin()
else:
    print("IMU não conectado. Verifique a conexão e tente novamente.")
    exit()

if qwiicGPS.connected:
    print("GPS conectado!")
    qwiicGPS.begin()
else:
    print("GPS não conectado. Verifique a conexão e tente novamente.")
    exit()

x= threading.Thread(target=_getData, args=(1,))
y= threading.Thread(target=run_example, args=(1,))

x.start()
y.start()

while(a>0):
    #run_example()
    plot_orientacao()