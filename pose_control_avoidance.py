import sim
import sys
import numpy as np
import keyboard
import time
import csv
import m_p3dx as p3dx

# ====================================================================
#  POSE CONTROL + OBJECT AVOIDANCE (BRAITENBERG)
# --------------------------------------------------------------------
#  Dua perilaku dijalankan bersamaan lalu dicampur (blending):
#    1. Pose control   -> kecepatan roda menuju Disc  (vel_goal)
#    2. Braitenberg    -> kecepatan roda menjauhi objek (vel_avoid)
#
#  alpha = aktivasi sensor depan terbesar (0..1)
#  WheelVel = (1 - alpha) * vel_goal + alpha * vel_avoid
#
#  Tidak ada objek  -> alpha = 0 -> murni pose control
#  Objek sangat dekat -> alpha = 1 -> murni Braitenberg
# ====================================================================

#++ parameter yang diubah tiap percobaan
KP      = [0.15, 0.15, 0.33]
VNORM   = 3.0
E_TOL   = 0.03
MODE_C1 = True
NAMA    = "T4_avoid"                 # nama file log

#? Parameter Braitenberg
noDetectionDist  = 0.5   # [m] di atas jarak ini objek diabaikan
maxDetectionDist = 0.2   # [m] di bawah jarak ini sensor aktif penuh
V0_AVOID = VNORM         # [rad/s] kecepatan dasar roda saat menghindar

# Sensor 0 = paling kiri ... 7 = paling kanan, 8..15 belakang (tidak dipakai)
# Objek di kiri -> roda kanan diperlambat -> belok kanan, dan sebaliknya
braitenbergL = np.array([-0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6,
                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
braitenbergR = np.array([-1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2,
                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
GAIN_AVOID = VNORM / 2.0  # skala bobot supaya sebanding dengan VNORM

def connect():
    sim.simxFinish(-1) # just in case, close all opened connections

    clientID=sim.simxStart('127.0.0.1',1999,True,True,5000,5) # Connect to CoppeliaSim

    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect')

    return clientID

def getObjectHandle(clientID, objectName):
    return sim.simxGetObjectHandle(clientID, objectName, sim.simx_opmode_blocking)[1]

def getObjectPosition(clientID, objectHandle):
    _, position = sim.simxGetObjectPosition(clientID, objectHandle, -1, sim.simx_opmode_blocking)
    _, orientation = sim.simxGetObjectOrientation(clientID, objectHandle, -1, sim.simx_opmode_blocking)
    ret_pos = np.array([position[0], position[1], orientation[2]])

    return ret_pos



def calculatePose(robot_pos, disc_pos, KP):
    #? Calculate the error
    error = np.array([disc_pos[0] - robot_pos[0], disc_pos[1] - robot_pos[1], disc_pos[2] - robot_pos[2]])

    #? Make sure the error is between -pi and pi
    if error[2] > np.pi:
        error[2] = error[2] - 2*np.pi
    elif error[2] < -np.pi:
        error[2] = error[2] + 2*np.pi

    #? Calculate the angle
    angle = np.arctan2(error[1], error[0]) - robot_pos[2]
    #? Make sure the angle is between -pi and pi
    if angle > np.pi:
        angle = angle - 2*np.pi
    elif angle < -np.pi:
        angle = angle + 2*np.pi

    rho = np.sqrt(error[0]**2 + error[1]**2)

    #? Physical parameters
    WheelRadius = 0.0975
    WidthBody = 0.28

    #? Matrix Forward Kinematics
    M_FK = np.array([[WheelRadius/2 * np.cos(robot_pos[2]),     WheelRadius/2 * np.cos(robot_pos[2])],
                     [WheelRadius/2 * np.sin(robot_pos[2]),     WheelRadius/2 * np.sin(robot_pos[2])],
                     [WheelRadius/(2*WidthBody),               -WheelRadius/(2*WidthBody)           ]
                    ])

    #? Matrix Pseudo Inverse
    M_PI = np.linalg.pinv(M_FK)

    #? Calculate the velocity
    error[0] = error[0] * KP[0]
    error[1] = error[1] * KP[1]

    #? Calculate distance
    distance = np.sqrt(error[0]**2 + error[1]**2)

    if distance > E_TOL:
        omega = angle * KP[2]
        fase1 = True
    else:
        omega = error[2] * KP[2]
        fase1 = False

    #? Make sure the omega is between -pi and pi
    if omega > np.pi:
        omega = omega - 2*np.pi
    elif omega < -np.pi:
        omega = omega + 2*np.pi


    #? v = K*rho*cos(theta - gamma), dibatasi >= 0 supaya robot tidak mundur
    if MODE_C1 and fase1:
        v = KP[0] * rho * np.cos(angle)
        if v < 0.0:
            v = 0.0
        vel = np.array([v/WheelRadius + WidthBody*omega/WheelRadius,
                        v/WheelRadius - WidthBody*omega/WheelRadius])
    else:
        vel = np.dot(M_PI, np.array([error[0], error[1], omega]))

    #? return the velocity [phiR, phiL]
    return vel


def _max(s1,s2):
    if s1 > s2:
        return s1
    else:
        return s2

def speed_normalization(speed_input,vnorm):
    q_max  = _max(speed_input[0], speed_input[1])

    speed_norm_out = np.zeros(2)

    if q_max > vnorm:
        speed_norm_out[0] = (vnorm / q_max) * speed_input[0]
        speed_norm_out[1] = (vnorm / q_max) * speed_input[1]

    else:
        speed_norm_out[0] = speed_input[0]
        speed_norm_out[1] = speed_input[1]

    return speed_norm_out


def sensor_activation(sensor):
    #? Ubah jarak [m] menjadi aktivasi 0..1 (0 = bebas, 1 = sangat dekat)
    detect = np.zeros(16)
    for i in range(16):
        dist = sensor[i]
        if dist < noDetectionDist:
            if dist < maxDetectionDist:
                dist = maxDetectionDist
            detect[i] = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
    return detect

def braitenberg(detect):
    #? Kecepatan roda menghindar, format [phiR, phiL] sama dengan calculatePose
    phiL = V0_AVOID + GAIN_AVOID * np.dot(braitenbergL, detect)
    phiR = V0_AVOID + GAIN_AVOID * np.dot(braitenbergR, detect)
    return np.array([phiR, phiL])




## ------------ Main Program ----------------
print('Main Program Started')

#? Connect to the simulator
clientID = connect()

#? Get the position handle
positionHandle = p3dx.getPositionHandle(clientID)
discHandle = getObjectHandle(clientID, 'Disc')
motorHandle = p3dx.getMotorHandle(clientID)
sensorsHandle = p3dx.getSensorsHandle(clientID)


time_prev = time.time()

_WheelVel = [0.0, 0.0]

log = []
t_start = sim.simxGetLastCmdTime(clientID)



## ---------- Routine 50 Hz ----------------
while True:

    if(time.time() - time_prev > 0.02):
        time_prev = time.time()

        #? Get the position
        robot_pos = p3dx.getPosition(clientID, positionHandle)
        disc_pos = getObjectPosition(clientID, discHandle)

        #? Behaviour 1: pose control (menuju Disc)
        WheelVel_goal = calculatePose(robot_pos, disc_pos, KP)
        WheelVel_raw = WheelVel_goal.copy()
        WheelVel_goal = speed_normalization(WheelVel_goal, VNORM)

        #? Behaviour 2: Braitenberg (menghindari objek)
        sensorValue = p3dx.getDistance(clientID, sensorsHandle)
        detect = sensor_activation(sensorValue)
        WheelVel_avoid = braitenberg(detect)

        #? Blending: semakin dekat objek, semakin dominan Braitenberg
        alpha = np.max(detect[0:8])
        WheelVel = (1 - alpha) * WheelVel_goal + alpha * WheelVel_avoid

        #? Acceleration and Deceleration
        _WheelVel[0] = _WheelVel[0] * 0.85 + WheelVel[0] * 0.15
        _WheelVel[1] = _WheelVel[1] * 0.85 + WheelVel[1] * 0.15

        p3dx.setWheelVel(clientID, motorHandle, _WheelVel)

        t_sim = (sim.simxGetLastCmdTime(clientID) - t_start) / 1000.0
        log.append((round(t_sim,3), robot_pos[0], robot_pos[1], robot_pos[2],
                    disc_pos[0], disc_pos[1], disc_pos[2],
                    _WheelVel[0], _WheelVel[1], WheelVel_raw[0], WheelVel_raw[1],
                    alpha))


        print("%.2f %.2f %.2f || %.2f %.2f %.2f || alpha = %.2f" % (robot_pos[0], robot_pos[1], robot_pos[2], disc_pos[0], disc_pos[1], disc_pos[2], alpha))



    #? Press q to quit
    if keyboard.is_pressed('q'):
        break


p3dx.setWheelVel(clientID, motorHandle, [0.0, 0.0])
#? Stop the simulation
sim.simxFinish(clientID)

#? simpan log setelah loop berhenti
with open("log_%s.csv" % NAMA, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(("# K", KP[0], "Ktheta", KP[2], "vnorm", VNORM, "e_tol", E_TOL, "c1", int(MODE_C1)))
    w.writerow(("t","x","y","gamma","xref","yref","gref","phiR","phiL","phiR_raw","phiL_raw","alpha"))
    w.writerows(log)
print("Log tersimpan: log_%s.csv (%d baris)" % (NAMA, len(log)))

print('Main Program Ended')
