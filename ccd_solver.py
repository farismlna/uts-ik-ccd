"""ccd_solver.py - Cyclic Coordinate Descent. Tidak bergantung pada raylib maupun config.

Satu langkah pada sendi i (ccd_joint_step) adalah satu-satunya tempat aturan CCD ditulis;
ccd() hanya mengulanginya, dan mode "langkah demi langkah" di aplikasi memanggil
fungsi yang sama. Jadi apa yang diperlihatkan saat demo = apa yang dihitung algoritma.
"""
import math
from kinematics import fk, wrap_angle


def joint_vectors(base, lengths, angles, target, i):
    """Pivot sendi i dan posisi end effector saat ini (dihitung dengan FK)."""
    pts = fk(base, lengths, angles)
    return pts[i], pts[-1]


def joint_delta(pivot, end_effector, target):
    """Sudut putar sendi agar vektor a (pivot -> end effector) sejajar vektor b (pivot -> target).
        a x b = |a||b| sin(delta),   a . b = |a||b| cos(delta)   ->   delta = atan2(a x b, a . b)
    """
    ax, ay = end_effector[0] - pivot[0], end_effector[1] - pivot[1]
    bx, by = target[0] - pivot[0], target[1] - pivot[1]
    cross = ax * by - ay * bx
    dot = ax * bx + ay * by
    return math.atan2(cross, dot)


def ccd_joint_step(base, lengths, angles, target, i, limits=None):
    """Satu langkah CCD: putar sendi i (angles DIUBAH DI TEMPAT). Kembalikan delta yang dihitung."""
    pivot, end_effector = joint_vectors(base, lengths, angles, target, i)
    delta = joint_delta(pivot, end_effector, target)
    angles[i] = wrap_angle(angles[i] + delta)
    if limits and limits[i] is not None:          # batas sendi: clamp tiap langkah
        lo, hi = limits[i]
        angles[i] = min(max(angles[i], lo), hi)
    return delta


def distance_to_target(base, lengths, angles, target):
    ex, ey = fk(base, lengths, angles)[-1]
    return math.hypot(target[0] - ex, target[1] - ey)


def ccd(base, lengths, angles, target, max_iter=10, tol=1e-3, limits=None):
    """Ulangi CCD dari sendi TERLUAR ke terdalam sampai jarak < tol atau max_iter tercapai.
    Kembalikan (jumlah_iterasi, jarak_akhir, riwayat_jarak_per_iterasi)."""
    if max_iter < 1:
        raise ValueError("max_iter harus >= 1")
    history = []
    for _ in range(max_iter):
        for i in range(len(angles) - 1, -1, -1):
            ccd_joint_step(base, lengths, angles, target, i, limits)
        dist = distance_to_target(base, lengths, angles, target)
        history.append(dist)
        if dist < tol:
            break
    return len(history), history[-1], history
