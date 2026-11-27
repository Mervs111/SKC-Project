"""m_p3dx versi ROBOT JetRacer (kit mahasiswa), pengganti m_p3dx.py CoppeliaSim.

Dipakai oleh semua program di folder robot/. Modul ini membungkus lib/p3dx.py
dari kit (otomatis bisa di-import di ./ros.sh), jadi program CoppeliaSim cukup
diubah sedikit:

    CoppeliaSim                           Robot (folder ini)
    import sim                            (hapus)
    import keyboard                       k = p3dx.tombol()   ->  k == 'w'
    import m_p3dx as p3dx                 import m_p3dx as p3dx          (sama)
    clientID = connect()                  clientID = p3dx.connect()
    p3dx.getSensorsHandle / getDistance   sama (16 sonar virtual dari LiDAR)
    p3dx.getMotorHandle                   sama
    p3dx.setWheelVel(.., [phiR, phiL])    sama, urutan KANAN dulu seperti di Coppelia
    p3dx.robotWheelKine(.., [v, w])       sama
    p3dx.getPositionHandle / getPosition  dari /odom atau peta (lihat di bawah)
    sim.simxFinish(clientID)              p3dx.disconnect(clientID)

Kenapa tidak langsung `import p3dx` dari kit?
    Di m_p3dx Coppelia kalian, setWheelVel memberi veloCmd[1] ke motor KIRI,
    jadi urutannya [phiR, phiL]. lib/p3dx.py kit memakai [phiL, phiR]. Tanpa
    penukar di modul ini, Braitenberg dan pose control belok ke arah yang salah.

Posisi robot (pengganti posisi dari simulator):
    sumber="odom" (default): dari /odom (encoder roda, 50 Hz).
        (0, 0, 0) = posisi & arah robot saat getPositionHandle() dipanggil.
        x = maju, y = ke kiri, gamma = sudut hadap (+ = berlawanan jarum jam).
        Akurat untuk jarak pendek; makin jauh/lama makin melenceng (drift).
    sumber="map": dari TF map -> base_footprint (koordinat peta). Jalankan
        lokalisasi kit dulu: ./ros.sh ros2 launch /workspace/nav/localization.launch.py peta:=<nama>
"""
import math
import threading
import time

import numpy as np
from numpy.linalg import inv

import p3dx as _kit      # lib/p3dx.py dari kit (PYTHONPATH /opt/jetracer_lib di ./ros.sh)

TANPA_DETEKSI = _kit.TANPA_DETEKSI      # 2.0: nilai sonar kalau tidak ada objek

# ---- sama persis dengan kit ------------------------------------------------
connect = _kit.connect                  # clientID = p3dx.connect()
disconnect = _kit.disconnect            # robot berhenti + terminal dipulihkan
tombol = _kit.tombol                    # tombol yang ditekan ('w', 'a', ...) atau ''
lepasTombol = _kit.lepasTombol          # untuk tombol "sekali tekan" (ganti mode)
getSensorsHandle = _kit.getSensorsHandle
getDistance = _kit.getDistance          # numpy array 16 jarak [m], tidak ada deteksi = 2.0
sensorAge = _kit.sensorAge              # umur data sonar terakhir [s]
getMotorHandle = _kit.getMotorHandle


# ---- roda: urutan [phiR, phiL] seperti m_p3dx Coppelia ---------------------
def setWheelVel(clientID, motorsHandle, veloCmd):
    """veloCmd = [phiR, phiL] (rad/s), KANAN dulu, sama dengan m_p3dx Coppelia."""
    return _kit.setWheelVel(clientID, motorsHandle, [veloCmd[1], veloCmd[0]])


def robotWheelKine(clientID, motorHandle, Vel):
    """Sama persis dengan m_p3dx Coppelia: Vel = [v, w] -> roda [phiR, phiL].

    Matriksnya sengaja tidak diubah: robot meniru P3-DX di Coppelia, jadi nilai
    w yang sudah di-tuning di simulasi menghasilkan belokan yang sama di robot.
    """
    # Robot parameters
    r = 0.0975  # Wheel radius [m]
    L = 0.28    # widthbody

    # Robot kinematics
    r_2 = r / 2
    r_2L = r_2 * L

    matrixKine = np.array([[r_2,   r_2],
                           [r_2L, -r_2L]])

    matrixKineInv = inv(matrixKine)
    velWheel = np.dot(matrixKineInv, Vel)
    # Set wheel velocity
    setWheelVel(clientID, motorHandle, velWheel)

    return 0


# ---- posisi robot ----------------------------------------------------------
def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class _Pose(object):
    """Isi positionHandle. Tidak perlu dipakai langsung."""

    def __init__(self, node, sumber, timeout):
        self.sumber = sumber
        self.lock = threading.Lock()
        self.pose = None        # (x, y, gamma) terbaru di frame sumber
        self.waktu = 0.0        # time.monotonic() saat pose terakhir datang
        self.awal = None        # pose odom saat mulai -> menjadi (0, 0, 0)

        if sumber == "odom":
            from nav_msgs.msg import Odometry
            node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        elif sumber == "map":
            import tf2_ros
            self._tf2 = tf2_ros
            self.buf = tf2_ros.Buffer()
            self.tf = tf2_ros.TransformListener(self.buf, node)
        else:
            raise ValueError("sumber harus 'odom' atau 'map', bukan %r" % sumber)

        t0 = time.monotonic()
        while self._mentah() is None and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        p = self._mentah()
        if p is None:
            if sumber == "odom":
                raise RuntimeError("belum ada data /odom setelah %.0f s. Robot menyala? ROBOT_IP benar?" % timeout)
            raise RuntimeError("belum ada TF map -> base_footprint setelah %.0f s. "
                               "Lokalisasi kit sudah dijalankan?" % timeout)
        if sumber == "odom":
            self.awal = p
        print("Posisi robot dari %s, siap" % sumber)

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self.lock:
            self.pose = (p.x, p.y, _yaw(q))
            self.waktu = time.monotonic()

    def _mentah(self):
        if self.sumber == "map":
            import rclpy.time
            try:
                t = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            except self._tf2.TransformException:
                pass                                    # pakai pose terakhir
            else:
                p, q = t.transform.translation, t.transform.rotation
                with self.lock:
                    self.pose = (p.x, p.y, _yaw(q))
                    self.waktu = time.monotonic()
        with self.lock:
            return self.pose

    def baca(self):
        x, y, g = self._mentah()
        if self.awal is None:
            return np.array([x, y, g])
        # Odom: nyatakan relatif terhadap pose awal (awal = 0, 0, 0)
        x0, y0, g0 = self.awal
        dx, dy = x - x0, y - y0
        c, s = math.cos(g0), math.sin(g0)
        return np.array([c * dx + s * dy, -s * dx + c * dy, _wrap(g - g0)])


def getPositionHandle(clientID, sumber="odom", timeout=5.0):
    """Pengganti handle '/PioneerP3DX'. sumber = "odom" (default) atau "map"."""
    return _Pose(clientID.node, sumber, timeout)


def getPosition(clientID, positionHandle):
    """[x, y, gamma] robot: x, y [m], gamma [rad]. Sama formatnya dengan Coppelia."""
    return positionHandle.baca()


def getOrientation(clientID, positionHandle):
    """[0, 0, gamma] seperti simxGetObjectOrientation (hanya yaw yang bermakna)."""
    return [0.0, 0.0, float(positionHandle.baca()[2])]


def poseAge(clientID, positionHandle):
    """Umur data posisi terakhir [s]. Besar = /odom atau TF tidak datang lagi."""
    with positionHandle.lock:
        return time.monotonic() - positionHandle.waktu
