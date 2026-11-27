# SKC-Project

Program kendali robot Pioneer P3-DX untuk dua target:

| Folder | Untuk | Cara jalan |
|---|---|---|
| `/` (folder utama) | **CoppeliaSim** di Windows | `python KeyboardControl.py` dll. |
| `robot/` | **Robot AMR JetRacer** ([kit mahasiswa](https://github.com/ismarintan98/jetracer-kit-mahasiswa)) di Linux | lewat `./ros.sh` kit, lihat panduan di bawah |

Repo ini juga dipakai sebagai jalur transfer Windows ⇄ Linux: edit & `git push`
di satu PC, lalu `git pull` di PC lain.

**Isi README:**
- [Panduan Linux: step by step](#panduan-linux-step-by-step), dari Linux baru sampai robot jalan
- [Panduan Claude Code di Linux](#panduan-claude-code-di-linux), memasang Claude Code di terminal

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

Cara paling mudah supaya `git push` bisa jalan: login sekali dengan GitHub
CLI (lihat langkah C3 di [Panduan Claude Code](#panduan-claude-code-di-linux)).
Kalau tidak pakai GitHub CLI: saat `git push` diminta **password**, isi dengan
**Personal Access Token** GitHub, bukan password akun. Buat token di GitHub → Settings → Developer
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

---

# Panduan Claude Code di Linux

Claude Code = asisten AI di terminal yang bisa membaca, mengedit, dan
menjalankan kode di folder proyek. Sumber: dokumentasi resmi
[code.claude.com/docs/en/setup](https://code.claude.com/docs/en/setup).

## C0. Syarat

- **Akun Claude berbayar** (Pro, Max, Team, atau Enterprise), atau akun
  Claude Console (API). **Paket gratis claude.ai tidak termasuk Claude Code.**
- Ubuntu 20.04 atau lebih baru (atau Debian 10+), RAM minimal 4 GB, internet.
- Tidak perlu Node.js.

## C1. Pasang Claude Code

```bash
sudo apt update
sudo apt install -y curl git
curl -fsSL https://claude.ai/install.sh | bash
```

- Jangan pakai `sudo` untuk baris `curl ... | bash`.
- Installer tidak menampilkan progres saat mengunduh. Tunggu sampai selesai.
- Setelah selesai, **tutup terminal lalu buka terminal baru**.

## C2. Cek pemasangan

```bash
claude --version        # contoh keluaran: 2.1.xxx (Claude Code)
claude doctor           # diagnosis pemasangan
```

Kalau muncul `claude: command not found`, folder `~/.local/bin` belum masuk PATH:

```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
claude --version
```

## C3. Login GitHub (opsional, supaya `git push` lancar)

Supaya kamu dan Claude bisa `git push` dari Linux tanpa mengetik token:

```bash
sudo apt install -y gh
gh auth login
```

Pilih **GitHub.com** → **HTTPS** → **Yes** (authenticate Git) → **Login with
a web browser**. Lalu salin kode yang muncul ke browser.

## C4. Jalankan & login pertama kali

Jalankan Claude dari folder repo ini:

```bash
cd ~/jetracer-kit-mahasiswa/SKC-Project
claude --add-dir ~/jetracer-kit-mahasiswa      # --add-dir: Claude ikut bisa membaca isi kit (lib/p3dx.py, README kit)
```

Saat pertama kali:

1. Browser terbuka untuk login. Masuk dengan akun Claude kamu.
2. Kalau browser tidak terbuka, tekan **`c`** di terminal untuk menyalin link
   login, lalu tempel di browser.
3. Kalau browser menampilkan **kode**, tempel kode itu di terminal pada tulisan
   `Paste code here if prompted`.
4. Muncul `Login successful`. Tekan **Enter**.

Login cukup sekali. Berikutnya cukup `cd` ke folder repo, lalu jalankan `claude`.

## C5. Cara pakai

- Ketik permintaan biasa dalam bahasa Indonesia. Contoh:
  - *"baca README.md dan robot/README.md dulu, lalu cek koneksi ke robot"*
  - *"naikkan V_MIN di pose_control.py jadi 0.12 lalu push ke github"*
- Claude di Linux **tidak tahu percakapan di Windows**. Di awal sesi, minta
  dia membaca `README.md` dan `robot/README.md` dulu.
- Perintah penting:

| Perintah | Fungsi |
|---|---|
| `claude` | mulai sesi baru |
| `claude -c` | lanjutkan percakapan terakhir di folder ini |
| `/help` | daftar perintah |
| `/login`, `/logout` | ganti / keluar akun |
| `/exit` | keluar dari Claude |

**Penting untuk robot:**

- Claude boleh menjalankan perintah yang **tidak menggerakkan robot**, misalnya
  `./ros.sh ros2 topic list`, edit kode, `git pull` / `git push`.
- Program yang **menggerakkan robot jalankan sendiri** di terminal terpisah
  (langkah 5), supaya kamu bisa menekan `q` / Ctrl-C kapan saja.
- Program yang memakai tombol (`KeyboardControl.py`, object follower) juga
  tidak bisa dikendalikan dari dalam Claude, karena terminal Claude tidak
  meneruskan tombol keyboard.
- Saat Claude meminta izin menjalankan perintah, baca dulu perintahnya sebelum
  menyetujui.

## C6. Update & hapus

- Claude Code **update otomatis**. Update manual: `claude update`.
- Hapus:

```bash
rm -f ~/.local/bin/claude
rm -rf ~/.local/share/claude
rm -rf ~/.claude ~/.claude.json      # ikut menghapus setelan, login, dan riwayat percakapan
```

## C7. Kalau ada masalah

| Gejala | Solusi |
|---|---|
| `claude: command not found` | Langkah C2 (tambahkan `~/.local/bin` ke PATH), lalu buka terminal baru |
| Error SSL / sertifikat saat instal | `sudo apt-get update && sudo apt-get install ca-certificates`, lalu ulangi C1 |
| Instal berhenti dengan tulisan `Killed` | RAM kurang. Tutup aplikasi lain lalu ulangi |
| Login berhasil tapi ditolak / tidak ada akses | Akun gratis tidak termasuk Claude Code. Pakai akun Pro/Max/Team/Enterprise |
| Masih bingung | `claude doctor`, atau panduan resmi [code.claude.com/docs/en/troubleshoot-install](https://code.claude.com/docs/en/troubleshoot-install) |
