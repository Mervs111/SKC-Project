import os
import sys
import numpy as np
import time
import csv                                          #++
import m_p3dx as p3dx                               #[ROBOT] m_p3dx versi robot (folder ini); sim & keyboard tidak dipakai

# ====================================================================
#  POSE CONTROL, versi ROBOT JetRacer
#  Perubahan dari versi CoppeliaSim ditandai  #[ROBOT]
#
#  Jalankan:  python3 pose_control.py                 (pakai TARGET di bawah)
#             python3 pose_control.py 1.5 0.5 90      (x [m], y [m], gamma [derajat])
#  q = berhenti (log CSV tetap tersimpan, juga saat Ctrl-C)
# ====================================================================

#++ parameter yang diubah tiap percobaan
KP      = [0.15, 0.15, 0.33]   # B1: 0.2 / 0.5 / 1.0 / 2.0
VNORM   = 3.0                  # B2: 2 / 4 / 8
E_TOL   = 0.03                 # B3: 0.01 / 0.05 / 0.2
MODE_C1 = True                # True untuk Bagian C1
NAMA    = "T2_C1_robot"           # nama file log

#[ROBOT] Target pengganti objek 'Disc' (di dunia nyata tidak ada Disc).
#        Dengan SUMBER_POSE = "odom", koordinatnya relatif terhadap posisi robot
#        saat program mulai: x = maju [m], y = ke kiri [m], gamma = arah akhir
#        [rad, + = kiri]. Dengan "map": koordinat peta (lihat nav/posisi_robot.py kit).
TARGET      = [1.5, 0.5, np.deg2rad(0.0)]
SUMBER_POSE = "odom"           #[ROBOT] "odom" (encoder) atau "map" (perlu lokalisasi kit jalan)

#[ROBOT] Mobil Ackermann tidak bisa berputar di tempat: v = 0 -> robot diam,
#        walaupun omega besar. Di Coppelia, saat target di samping/belakang
#        (cos(angle) <= 0) v dibuat 0 dan P3-DX berputar di tempat; di robot itu
#        membuat robot macet. Selama fase 1 kecepatan maju dijaga >= V_MIN.
V_MIN     = 0.10               # [m/s]
POSE_BASI = 0.5                # [s] data posisi lebih tua dari ini -> robot berhenti

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

    rho = np.sqrt(error[0]**2 + error[1]**2)        #++ jarak asli ke target, untuk C1

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

    if distance > E_TOL:                            #++ dulu 0.03
        omega = angle * KP[2]
        fase1 = True                                #++
    else:
        omega = error[2] * KP[2]
        fase1 = False                               #++

    #? Make sure the omega is between -pi and pi
    if omega > np.pi:
        omega = omega - 2*np.pi
    elif omega < -np.pi:
        omega = omega + 2*np.pi


    #++ ---------- BAGIAN C1 ----------
    #++ v = K*rho*cos(theta - gamma), dibatasi >= 0 supaya robot tidak mundur
    if MODE_C1 and fase1:
        v = KP[0] * rho * np.cos(angle)
        if v < 0.0:
            v = 0.0
        #++ invers kinematika dari M_FK:  v = (r/2)(phiR+phiL) , omega = r/(2L)(phiR-phiL)
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


def jaga_kecepatan_minimum(vel, v_min):             #[ROBOT]
    # Naikkan kecepatan maju v = r*(phiR+phiL)/2 sampai v_min, tanpa mengubah
    # selisih roda (phiR - phiL), jadi omega tetap sama.
    WheelRadius = 0.0975
    phi_min = v_min / WheelRadius
    rata = (vel[0] + vel[1]) / 2.0
    if rata < phi_min:
        vel = vel + (phi_min - rata)
    return vel


def simpan_log(log):
    # [ROBOT] disimpan di folder program ini (bukan folder kerja container)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log_%s.csv" % NAMA)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(("# K", KP[0], "Ktheta", KP[2], "vnorm", VNORM, "e_tol", E_TOL, "c1", int(MODE_C1),
                    "vmin", V_MIN, "sumber", SUMBER_POSE))
        w.writerow(("t","x","y","gamma","xref","yref","gref","phiR","phiL","phiR_raw","phiL_raw"))
        w.writerows(log)
    print("Log tersimpan: %s (%d baris)" % (path, len(log)))   #++


## ------------ Main Program ----------------
print('Main Program Started')

#? Connect to the robot
clientID = p3dx.connect()                           #[ROBOT] pengganti sim.simxStart

#? Get the position handle
positionHandle = p3dx.getPositionHandle(clientID, SUMBER_POSE)   #[ROBOT] dari /odom atau peta
disc_pos = np.array(TARGET, dtype=float)            #[ROBOT] pengganti posisi 'Disc'
motorHandle = p3dx.getMotorHandle(clientID)

print("Target: x = %.2f m, y = %.2f m, gamma = %.0f deg (pose dari %s). Tekan q untuk berhenti."
      % (disc_pos[0], disc_pos[1], np.rad2deg(disc_pos[2]), SUMBER_POSE))

time_prev = time.time()

_WheelVel = [0.0, 0.0]

log = []                                            #++ sebelum loop
t_start = time.monotonic()                          #[ROBOT] pengganti sim.simxGetLastCmdTime
t_print = 0.0


## ---------- Routine 50 Hz ----------------
try:
    while True:

        if(time.time() - time_prev > 0.02):
            time_prev = time.time()

            #? Get the position
            robot_pos = p3dx.getPosition(clientID, positionHandle)

            WheelVel, fase1 = calculatePose(robot_pos, disc_pos, KP)

            WheelVel_raw = WheelVel.copy()          #++ perintah sebelum normalisasi

            #? Speed Normalization
            WheelVel = speed_normalization(WheelVel, VNORM)

            #[ROBOT] Ackermann: selama fase 1 robot harus tetap maju supaya bisa belok
            if fase1:
                WheelVel = jaga_kecepatan_minimum(WheelVel, V_MIN)

            #? Acceleration and Deceleration
            _WheelVel[0] = _WheelVel[0] * 0.85 + WheelVel[0] * 0.15
            _WheelVel[1] = _WheelVel[1] * 0.85 + WheelVel[1] * 0.15

            #[ROBOT] /odom atau TF berhenti datang -> jangan jalan buta
            if p3dx.poseAge(clientID, positionHandle) > POSE_BASI:
                _WheelVel = [0.0, 0.0]
                print("!! data posisi basi, robot berhenti")

            p3dx.setWheelVel(clientID, motorHandle, _WheelVel)

            #++ di akhir tiap iterasi
            t_sim = time.monotonic() - t_start      #[ROBOT] waktu nyata sejak mulai
            log.append((round(t_sim,3), robot_pos[0], robot_pos[1], robot_pos[2],
                        disc_pos[0], disc_pos[1], disc_pos[2],
                        _WheelVel[0], _WheelVel[1], WheelVel_raw[0], WheelVel_raw[1]))

            if time.monotonic() - t_print > 0.2:    #[ROBOT] print 5x per detik
                t_print = time.monotonic()
                print("%.2f %.2f %.2f || %.2f %.2f %.2f || fase %d"
                      % (robot_pos[0], robot_pos[1], robot_pos[2], disc_pos[0], disc_pos[1], disc_pos[2],
                         1 if fase1 else 2))

        #? Press q to quit
        if p3dx.tombol() == 'q':                    #[ROBOT] pengganti keyboard.is_pressed('q')
            break
        time.sleep(0.002)                           #[ROBOT] supaya tidak makan 100% CPU

finally:
    #[ROBOT] selalu dijalankan, juga saat Ctrl-C atau error: robot berhenti & log tersimpan
    p3dx.setWheelVel(clientID, motorHandle, [0.0, 0.0])
    p3dx.disconnect(clientID)                       #[ROBOT] pengganti sim.simxFinish
    simpan_log(log)                                 #++ simpan log setelah loop berhenti

print('Main Program Ended')
