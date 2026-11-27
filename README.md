# SKC-Project

Program kendali robot Pioneer P3-DX untuk dua target:

| Folder | Untuk | Cara jalan |
|---|---|---|
| `/` (folder utama) | **CoppeliaSim** di Windows | `python KeyboardControl.py` dll. |
| `robot/` | **Robot AMR JetRacer** ([kit mahasiswa](https://github.com/ismarintan98/jetracer-kit-mahasiswa)) di Linux | lewat `./ros.sh` kit, lihat panduan di bawah |

Repo ini juga dipakai sebagai jalur transfer Windows ⇄ Linux: edit & `git push`
di satu PC, lalu `git pull` di PC lain.

---

# Panduan Linux: step by step

Panduan ini dari Linux yang baru dipasang sampai robot jalan. Langkah 1–4
cukup **sekali saja**. Untuk sesi berikutnya langsung ke **langkah 5**.

## 0. Syarat

- **Linux asli** (disarankan Ubuntu 22.04 / 24.04, dual-boot atau PC lab).
  - **Tidak bisa:** WSL dan Docker Desktop, karena robot tidak terjangkau dari sana.
  - Kalau Linux-nya di VirtualBox/VMware, network adapter harus **Bridged**,
    bukan NAT.
- Ruang kosong **± 8 GB** dan internet untuk unduh ± 5 GB (sekali saja).
- User dengan hak `sudo`.
- PC tersambung ke **WiFi/LAN yang sama dengan robot**. Tanya dosen nama
  jaringannya dan subnetnya (default kit: `10.17.36.0/22`).

## 1. Pasang git & ambil kit robot

Buka Terminal (Ctrl+Alt+T):

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/ismarintan98/jetracer-kit-mahasiswa.git ~/jetracer-kit-mahasiswa
```

## 2. Jalankan setup kit (sekali, 10–30 menit)

```bash
cd ~/jetracer-kit-mahasiswa
./setup.sh                          # subnet robot default 10.17.36.0/22
# ./setup.sh --subnet <subnet>      # kalau subnet jaringan robot beda (tanya dosen)
```

- Jalankan **tanpa** `sudo`. Script akan meminta password sendiri.
- Script ini memasang Docker, membangun image ROS 2, dan membuka firewall.
- Kalau di akhir muncul `PENTING: LOGOUT lalu LOGIN lagi`: **logout lalu
  login lagi**, atau reboot. Menutup terminal saja tidak cukup.

## 3. Sambungkan ke robot

1. Nyalakan robot. Tunggu ± 1 menit sampai **IP robot muncul di layar OLED**.
2. Cek dari PC (ganti `10.17.36.90` dengan IP di OLED):

```bash
cd ~/jetracer-kit-mahasiswa
ping -c 3 10.17.36.90               # harus ada balasan
export ROBOT_IP=10.17.36.90
./ros.sh ros2 topic list
```

Harus muncul antara lain `/odom`, `/scan`, `/sonar`, `/cmd_wheel_vel`.
Kalau hanya `/parameter_events` dan `/rosout`, lihat bagian **Kalau ada masalah**.

Supaya `ROBOT_IP` tidak perlu diketik terus:

```bash
echo 'export ROBOT_IP=10.17.36.90' >> ~/.bashrc     # buka terminal baru setelah ini
```

IP robot bisa berganti. Selalu cocokkan dengan OLED, dan ubah baris itu di
`~/.bashrc` kalau berbeda.

## 4. Ambil repo ini (di DALAM folder kit)

```bash
cd ~/jetracer-kit-mahasiswa
git clone https://github.com/Mervs111/SKC-Project.git
```

Repo ini harus ada di dalam folder kit, karena `./ros.sh` hanya bisa melihat
isi folder kit (di dalam container namanya `/workspace`). Jadi
`~/jetracer-kit-mahasiswa/SKC-Project/robot/` = `/workspace/SKC-Project/robot/`.

## 5. Jalankan program

**Percobaan pertama: angkat roda robot dari lantai** (taruh di atas kotak).

Semua perintah dijalankan dari folder kit:

```bash
cd ~/jetracer-kit-mahasiswa
git -C SKC-Project pull             # ambil versi kode terbaru dulu
```

Urutan percobaan yang disarankan:

| # | Program | Perintah | Tombol |
|---|---|---|---|
| 1 | Keyboard | `./ros.sh python3 /workspace/SKC-Project/robot/KeyboardControl.py` | `w` maju, `s` mundur, `a`/`d` maju belok, `q` keluar |
| 2 | Object follower | `./ros.sh python3 /workspace/SKC-Project/robot/ObjectFollower_Forward+Reverse.py` | `w/s/a/d` manual, `c` ikuti depan, `v` ikuti belakang, `q` |
| 3 | Braitenberg | `./ros.sh python3 /workspace/SKC-Project/robot/ObjectAvoidance_Braitenberg.py` | robot jalan sendiri, `q` berhenti |
| 4 | Pose control | `./ros.sh python3 /workspace/SKC-Project/robot/pose_control.py 1.5 0.5 0` | `q` berhenti |
| 5 | Pose + hindar | `./ros.sh python3 /workspace/SKC-Project/robot/pose_control_avoidance.py 2.0 0 0` | `q` berhenti |

Catatan:

- **Tahan tombolnya.** Robot berhenti ± 0.6 s setelah tombol dilepas. Untuk
  `c`/`v`, cukup tekan sekali.
- Klik dulu jendela terminalnya supaya tombol terbaca.
- **Pose control:** angka di belakang nama program = target `x y gamma`.
  - `x` = maju [m], `y` = ke kiri [m], `gamma` = arah akhir [derajat].
  - Semua relatif terhadap posisi robot **saat program dimulai**.
  - Contoh: `1.5 0.5 0` = 1.5 m ke depan, 0.5 m ke kiri. Tanpa angka,
    yang dipakai `TARGET` di dalam file.
  - Sisakan ruang kosong ± 3 × 3 m.
- `Ctrl-C` juga aman: robot berhenti dan log tetap tersimpan.
- Sonar baru terisi ± 1.5 detik setelah program mulai. Itu normal.
- **Satu robot, satu pengendali.** Jangan ada dua PC yang menjalankan program
  ke robot yang sama bersamaan.

Program **wajib** dijalankan lewat `./ros.sh`. `python3 file.py` langsung di
terminal Linux akan gagal (`No module named 'p3dx'` / `rclpy`).

## 6. Edit kode & transfer Windows ⇄ Linux

**Edit di Linux:** buka file di `~/jetracer-kit-mahasiswa/SKC-Project/robot/`
dengan editor apa saja (VS Code, gedit, ...). Simpan, lalu jalankan lagi
dengan `./ros.sh`. Tidak perlu build ulang.

**Edit di Windows → jalankan di Linux:**

```bash
# Windows:  git add . ; git commit -m "..." ; git push
# Linux:
cd ~/jetracer-kit-mahasiswa/SKC-Project && git pull
```

**Kirim hasil dari Linux ke Windows** (mis. log `robot/log_*.csv` untuk laporan):

```bash
cd ~/jetracer-kit-mahasiswa/SKC-Project
git config --global user.name  "Nama Kamu"          # sekali saja
git config --global user.email "email-github@kamu"  # sekali saja
git add robot/log_*.csv
git commit -m "Log percobaan robot"
git push
```

Saat `git push` diminta **password**, isi dengan **Personal Access Token**
GitHub, bukan password akun. Buat token di GitHub → Settings → Developer
settings → Personal access tokens → *Fine-grained* (izin **Contents: Read and
write** untuk repo ini). Simpan tokennya supaya tidak perlu diketik ulang:
`git config --global credential.helper store`.

Lalu di Windows: `git pull`.

## Kalau ada masalah

| Gejala | Solusi |
|---|---|
| `permission denied while trying to connect to the docker API` | Belum logout/login setelah `setup.sh`. Logout → login, atau sementara jalankan `newgrp docker` di terminal itu |
| `ros2 topic list` hanya `/parameter_events` dan `/rosout` | IP robot salah (cek OLED), beda jaringan (`ping`), atau subnet firewall salah: `./setup.sh --subnet <subnet robot>` |
| `!! ROBOT_IP belum diisi` | `export ROBOT_IP=<IP dari OLED>` |
| `belum ada data /sonar` atau `/odom` | Robot belum selesai boot / IP salah. Tunggu, lalu jalankan lagi |
| `No module named 'p3dx'` | Program dijalankan tanpa `./ros.sh` |
| `No such file` untuk `/workspace/SKC-Project/...` | Repo ini belum di-clone di dalam folder kit (langkah 4) |
| Tombol tidak bereaksi | Klik jendela terminalnya dulu; jalankan dari terminal biasa, bukan dari tombol "Run" editor |
| Robot tidak bergerak saat `a`/`d` di depan objek | Normal: rem anti-nabrak memblokir maju. Mundur dengan `s` |
| Robot berhenti-berhenti sendiri | Sinyal WiFi lemah. Dekatkan PC ke robot / router |
| Robot tidak mau jalan pelan saat pose control | Naikkan `V_MIN` di file (mis. 0.12–0.15) |

Detail perubahan program dari versi CoppeliaSim: [`robot/README.md`](robot/README.md).
Panduan lengkap kit (RViz, SLAM, peta, Nav2): `~/jetracer-kit-mahasiswa/README.md`.
