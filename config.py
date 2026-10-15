"""config.py - Semua konstanta aplikasi di satu tempat (mudah diubah tanpa menyentuh algoritma)."""
import math

# --- Lengan 3 segmen ---
LENGTHS = [130.0, 100.0, 70.0]         # panjang tiap segmen (piksel), total 300
START_ANGLES = [0.5, 0.5, 0.5]         # pose awal (radian); sengaja TIDAK lurus
BASE = (0.0, 0.0)                      # pangkal lengan di koordinat dunia

# --- Batas sendi (aktif dengan tombol L); None = bebas ---
_LIM = math.radians(100)
LIMITS_ON = [None, (-_LIM, _LIM), (-_LIM, _LIM)]

# --- Parameter CCD ---
MAX_ITER_PER_FRAME = 10                # iterasi CCD per frame (mode otomatis)
TOL = 1e-3                             # toleransi jarak end effector ke target

# --- Jendela ---
WIN_W, WIN_H = 800, 600
ORIGIN = (WIN_W // 2, WIN_H // 2)      # posisi pangkal di layar
