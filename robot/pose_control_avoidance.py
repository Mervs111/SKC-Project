import os
import sys
import numpy as np
import time
import csv
import m_p3dx as p3dx          #[ROBOT] m_p3dx versi robot (folder ini); sim & keyboard tidak dipakai

# ====================================================================
#  POSE CONTROL + OBJECT AVOIDANCE (BRAITENBERG), versi ROBOT JetRacer
#  Perubahan dari versi CoppeliaSim ditandai  #[ROBOT]
# --------------------------------------------------------------------
#  Dua perilaku dijalankan bersamaan lalu dicampur (blending):
#    1. Pose control   -> kecepatan roda menuju target (vel_goal)
#    2. Braitenberg    -> kecepatan roda menjauhi objek (vel_avoid)
#
#  alpha = aktivasi sensor depan terbesar (0..1)
#  WheelVel = (1 - alpha) * vel_goal + alpha * vel_avoid
#
#  Tidak ada objek  -> alpha = 0 -> murni pose control
#  Objek sangat dekat -> alpha = 1 -> murni Braitenberg
#
#  [ROBOT] Ditambah: kecepatan maju minimum (Ackermann tidak bisa berputar
#  di tempat) dan "rem & mundur" kalau objek terlalu dekat di depan.
#
#  Jalankan:  python3 pose_control_avoidance.py              (pakai TARGET di bawah)
#             python3 pose_control_avoidance.py 2.0 0 0      (x [m], y [m], gamma [derajat])
#  q = berhenti (log CSV tetap tersimpan, juga saat Ctrl-C)
# ====================================================================

#++ parameter yang diubah tiap percobaan
KP      = [0.15, 0.15, 0.33]
VNORM   = 3.0
E_TOL   = 0.03
MODE_C1 = True
NAMA    = "T4_avoid_robot"           # nama file log

#[ROBOT] Target pengganti objek 'Disc'. Dengan SUMBER_POSE = "odom":
#        relatif posisi awal robot, x = maju [m], y = ke kiri [m], gamma [rad].
TARGET      = [2.0, 0.0, np.deg2rad(0.0)]
SUMBER_POSE = "odom"     #[ROBOT] "odom" (encoder) atau "map" (perlu lokalisasi kit jalan)

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

#[ROBOT] Batasan robot Ackermann
V_MIN        = 0.10      # [m/s] kecepatan maju minimum selama fase 1 (supaya bisa belok)
JARAK_MUNDUR = 0.15      # [m] objek di depan (sensor 2..5) lebih dekat dari ini -> mundur
MUNDUR_DETIK = 1.5       # [s] lama mundur
KUNCI_DETIK  = 3.0       # [s] mundur lagi dalam waktu ini -> arah belok sama dengan sebelumnya
PHI_MUNDUR   = 1.5       # [rad/s] kecepatan roda saat mundur (~0.15 m/s)
POSE_BASI    = 0.5       # [s] data posisi lebih tua dari ini -> robot berhenti
SONAR_BASI   = 1.0       # [s] data sonar lebih tua dari ini -> robot berhenti

if len(sys.argv) >= 3:                              #[ROBOT] target dari argumen
    TARGET = [float(sys.argv[1]), float(sys.argv[2]),
              np.deg2rad(float(sys.argv[3])) if len(sys.argv) >= 4 else 0.0]


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

    #? return the velocity [phiR, phiL] dan fase    #[ROBOT] fase1 ikut dikembalikan
    return vel, fase1


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


def jaga_kecepatan_minimum(vel, v_min):             #[ROBOT]
    # Naikkan kecepatan maju v = r*(phiR+phiL)/2 sampai v_min, tanpa mengubah
    # selisih roda (phiR - phiL), jadi omega tetap sama.
    WheelRadius = 0.0975
    phi_min = v_min / WheelRadius
    rata = (vel[0] + vel[1]) / 2.0
    if rata < phi_min:
        vel = vel + (phi_min - rata)
    return vel

def pilih_arah_mundur(sensor):                      #[ROBOT]
    # Objek lebih dekat di kiri -> moncong diputar ke kanan (-1), dan sebaliknya.
    return -1.0 if min(sensor[0:4]) < min(sensor[4:8]) else 1.0

def kecepatan_mundur(arah):                         #[ROBOT]
    # Mundur sambil memutar moncong: arah +1 = kiri, -1 = kanan. Satu roda 0,
    # roda lain -2*PHI_MUNDUR = belok paling tajam. Format [phiR, phiL].
    dphi = PHI_MUNDUR * arah
    return np.array([-PHI_MUNDUR + dphi, -PHI_MUNDUR - dphi])


def simpan_log(log):
    # [ROBOT] disimpan di folder program ini (bukan folder kerja container)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log_%s.csv" % NAMA)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(("# K", KP[0], "Ktheta", KP[2], "vnorm", VNORM, "e_tol", E_TOL, "c1", int(MODE_C1),
                    "vmin", V_MIN, "sumber", SUMBER_POSE))
        w.writerow(("t","x","y","gamma","xref","yref","gref","phiR","phiL","phiR_raw","phiL_raw","alpha","mundur"))
        w.writerows(log)
    print("Log tersimpan: %s (%d baris)" % (path, len(log)))


## ------------ Main Program ----------------
print('Main Program Started')

#? Connect to the robot
clientID = p3dx.connect()                           #[ROBOT] pengganti sim.simxStart

#? Get the position handle
positionHandle = p3dx.getPositionHandle(clientID, SUMBER_POSE)   #[ROBOT] dari /odom atau peta
disc_pos = np.array(TARGET, dtype=float)            #[ROBOT] pengganti posisi 'Disc'
motorHandle = p3dx.getMotorHandle(clientID)
sensorsHandle = p3dx.getSensorsHandle(clientID)

print("Target: x = %.2f m, y = %.2f m, gamma = %.0f deg (pose dari %s). Tekan q untuk berhenti."
      % (disc_pos[0], disc_pos[1], np.rad2deg(disc_pos[2]), SUMBER_POSE))

time_prev = time.time()

_WheelVel = [0.0, 0.0]
mundur_sampai = -1e9                                #[ROBOT]
arah_mundur = 1.0

log = []
t_start = time.monotonic()                          #[ROBOT] pengganti sim.simxGetLastCmdTime
t_print = 0.0


## ---------- Routine 50 Hz ----------------
try:
    while True:

        if(time.time() - time_prev > 0.02):
            time_prev = time.time()
            now = time.monotonic()

            #? Get the position
            robot_pos = p3dx.getPosition(clientID, positionHandle)

            #? Behaviour 1: pose control (menuju target)
            WheelVel_goal, fase1 = calculatePose(robot_pos, disc_pos, KP)
            WheelVel_raw = WheelVel_goal.copy()
            WheelVel_goal = speed_normalization(WheelVel_goal, VNORM)

            #? Behaviour 2: Braitenberg (menghindari objek)
            sensorValue = p3dx.getDistance(clientID, sensorsHandle)
            detect = sensor_activation(sensorValue)
            WheelVel_avoid = braitenberg(detect)

            #? Blending: semakin dekat objek, semakin dominan Braitenberg
            alpha = np.max(detect[0:8])
            WheelVel = (1 - alpha) * WheelVel_goal + alpha * WheelVel_avoid

            #[ROBOT] Ackermann: selama fase 1 robot harus tetap maju supaya bisa belok
            if fase1:
                WheelVel = jaga_kecepatan_minimum(WheelVel, V_MIN)

            #? Acceleration and Deceleration
            _WheelVel[0] = _WheelVel[0] * 0.85 + WheelVel[0] * 0.15
            _WheelVel[1] = _WheelVel[1] * 0.85 + WheelVel[1] * 0.15

            #[ROBOT] Rem & mundur kalau objek terlalu dekat di depan (langsung, tanpa filter)
            status = ''
            if min(sensorValue[2:6]) < JARAK_MUNDUR and now >= mundur_sampai:
                if now - mundur_sampai > KUNCI_DETIK:
                    arah_mundur = pilih_arah_mundur(sensorValue)
                mundur_sampai = now + MUNDUR_DETIK
                vel_mundur = kecepatan_mundur(arah_mundur)
            mundur = now < mundur_sampai
            if mundur:
                if min(sensorValue[10:14]) < JARAK_MUNDUR:
                    _WheelVel = [0.0, 0.0]          # belakang juga tertutup: diam
                    status = 'TERJEPIT'
                else:
                    _WheelVel = list(vel_mundur)
                    status = 'MUNDUR'

            #[ROBOT] posisi atau sonar berhenti datang -> jangan jalan buta
            if (p3dx.poseAge(clientID, positionHandle) > POSE_BASI
                    or p3dx.sensorAge(clientID) > SONAR_BASI):
                _WheelVel = [0.0, 0.0]
                status = 'DATA BASI, berhenti'

            p3dx.setWheelVel(clientID, motorHandle, _WheelVel)

            t_sim = time.monotonic() - t_start      #[ROBOT] waktu nyata sejak mulai
            log.append((round(t_sim,3), robot_pos[0], robot_pos[1], robot_pos[2],
                        disc_pos[0], disc_pos[1], disc_pos[2],
                        _WheelVel[0], _WheelVel[1], WheelVel_raw[0], WheelVel_raw[1],
                        alpha, int(mundur)))

            if now - t_print > 0.2:                 #[ROBOT] print 5x per detik
                t_print = now
                print("%.2f %.2f %.2f || %.2f %.2f %.2f || alpha = %.2f %s"
                      % (robot_pos[0], robot_pos[1], robot_pos[2], disc_pos[0], disc_pos[1], disc_pos[2],
                         alpha, status))

        #? Press q to quit
        if p3dx.tombol() == 'q':                    #[ROBOT] pengganti keyboard.is_pressed('q')
            break
        time.sleep(0.002)                           #[ROBOT] supaya tidak makan 100% CPU

finally:
    #[ROBOT] selalu dijalankan, juga saat Ctrl-C atau error: robot berhenti & log tersimpan
    p3dx.setWheelVel(clientID, motorHandle, [0.0, 0.0])
    p3dx.disconnect(clientID)                       #[ROBOT] pengganti sim.simxFinish
    simpan_log(log)                                 #? simpan log setelah loop berhenti

print('Main Program Ended')
