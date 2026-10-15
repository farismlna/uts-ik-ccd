"""kinematics.py - Forward kinematics dan utilitas geometri. Tidak bergantung pada raylib.

Konvensi (sama dengan slide P10-P11):
  - sumbu y ke ATAS, sudut positif = berlawanan arah jarum jam
  - sudut sendi LOKAL (relatif terhadap segmen sebelumnya), satuan radian
"""
import math


def wrap_angle(a):
    """Bungkus sudut ke (-pi, pi] agar nilainya tidak terus membesar."""
    return math.atan2(math.sin(a), math.cos(a))


def fk(base, lengths, angles):
    """Posisi semua sendi. Elemen ke-i = pivot sendi i, elemen terakhir = end effector.
    Sudut lokal diakumulasi menjadi sudut global: phi_i = theta_1 + ... + theta_i."""
    x, y = base
    phi = 0.0
    points = [(x, y)]
    for length, theta in zip(lengths, angles):
        phi += theta
        x += length * math.cos(phi)
        y += length * math.sin(phi)
        points.append((x, y))
    return points


def reach_range(lengths):
    """(jangkauan_minimum, jangkauan_maksimum) end effector dari pangkal, tanpa batas sendi."""
    total = sum(lengths)
    return max(0.0, 2 * max(lengths) - total), total
