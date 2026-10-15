"""test_ik.py - Uji mandiri tanpa raylib. Jalankan:  python test_ik.py"""
import math
import sys

import config as cfg
from kinematics import fk, wrap_angle, reach_range
from ccd_solver import ccd, ccd_joint_step, joint_vectors, joint_delta, distance_to_target

ok = True


def check(name, cond, detail=""):
    global ok
    ok = ok and cond
    print(f"[{'OK' if cond else 'GAGAL'}] {name} {detail}")


# 1. FK: contoh slide P10, L=(2,1), theta=(30, 45) derajat -> sekitar (1.991, 1.966)
ex, ey = fk((0, 0), [2, 1], [math.radians(30), math.radians(45)])[-1]
check("FK contoh slide P10", abs(ex - 1.991) < 1e-3 and abs(ey - 1.966) < 1e-3, f"-> ({ex:.3f}, {ey:.3f})")

# 2. Jangkauan lengan aplikasi
r_min, r_max = reach_range(cfg.LENGTHS)
check("Jangkauan lengan aplikasi", r_min == 0.0 and r_max == 300.0, f"-> ({r_min}, {r_max})")

# 3. Satu langkah CCD pada kasus yang bisa dihitung tangan:
#    lengan lurus sepanjang x (1+1), target (0,2): delta sendi 1 = atan2(4, 0) = pi/2, ujung -> (0,2)
a = [0.0, 0.0]
pivot, ee = joint_vectors((0, 0), [1, 1], a, (0, 2), 0)
d = joint_delta(pivot, ee, (0, 2))
ccd_joint_step((0, 0), [1, 1], a, (0, 2), 0)
ex, ey = fk((0, 0), [1, 1], a)[-1]
check("Satu langkah CCD (hitung tangan): delta = pi/2",
      abs(d - math.pi / 2) < 1e-12 and abs(ex) < 1e-9 and abs(ey - 2) < 1e-9, f"-> ujung=({ex:.3f}, {ey:.3f})")

# 4. CCD contoh buku (Parent, Fig. 5.25), seperti slide P11
a = [math.pi / 8, math.pi / 4, math.pi / 4]
it, dist, hist = ccd((0, 0), [15, 10, 5], a, (-20, 5), max_iter=20, tol=1e-3)
deg = [math.degrees(v) for v in a]
check("CCD contoh buku: 4 iterasi, jarak sekitar 0.0007", it == 4 and abs(dist - 0.0007) < 1e-4,
      f"-> iterasi={it}, jarak={dist:.5f}")
check("CCD contoh buku: sudut akhir 146.3, 87.4, -125.1", all(abs(x - y) < 0.15 for x, y in zip(deg, [146.3, 87.4, -125.1])),
      f"-> {[round(v, 1) for v in deg]}")
check("Riwayat jarak menurun tiap iterasi", all(h2 < h1 for h1, h2 in zip(hist, hist[1:])),
      f"-> {[round(h, 4) for h in hist]}")

# 5. Target dalam jangkauan konvergen; di luar jangkauan lengan meregang
a = list(cfg.START_ANGLES)
it, dist, _ = ccd(cfg.BASE, cfg.LENGTHS, a, (120.0, 80.0), max_iter=50)
check("Target dalam jangkauan konvergen", dist < cfg.TOL, f"-> jarak={dist:.5f}")
a = list(cfg.START_ANGLES)
ccd(cfg.BASE, cfg.LENGTHS, a, (700.0, 0.0), max_iter=50)
ex, ey = fk(cfg.BASE, cfg.LENGTHS, a)[-1]
check("Target di luar jangkauan: lengan lurus ke arah target", abs(ex - 300) < 1e-3 and abs(ey) < 1e-3,
      f"-> ujung=({ex:.1f}, {ey:.1f})")

# 6. Batas sendi dipatuhi
a = list(cfg.START_ANGLES)
ccd(cfg.BASE, cfg.LENGTHS, a, (10.0, 10.0), max_iter=50, limits=cfg.LIMITS_ON)
check("Batas sendi dipatuhi", all(abs(a[i]) <= math.radians(100) + 1e-9 for i in (1, 2)),
      f"-> sudut={[round(math.degrees(v), 1) for v in a]}")

# 7. Satu iterasi ccd() == tiga langkah manual (terluar -> terdalam)
a1, a2 = list(cfg.START_ANGLES), list(cfg.START_ANGLES)
t = (150.0, -60.0)
ccd(cfg.BASE, cfg.LENGTHS, a1, t, max_iter=1)
for i in (2, 1, 0):
    ccd_joint_step(cfg.BASE, cfg.LENGTHS, a2, t, i)
check("ccd(max_iter=1) sama dengan langkah manual", all(abs(x - y) < 1e-12 for x, y in zip(a1, a2)))

print("\nSEMUA UJI LULUS" if ok else "\nADA UJI YANG GAGAL")
sys.exit(0 if ok else 1)
