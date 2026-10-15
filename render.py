"""render.py - Konversi koordinat dan semua fungsi menggambar (raylib). Tidak berisi algoritma IK."""
import math
import pyray as rl
import config as cfg


# --- Koordinat: dunia (y ke atas) <-> layar (y ke bawah) ---
def to_screen(p):
    return (cfg.ORIGIN[0] + p[0], cfg.ORIGIN[1] - p[1])


def to_world(sx, sy):
    return (sx - cfg.ORIGIN[0], cfg.ORIGIN[1] - sy)


def _line(a, b, thickness, color):
    rl.draw_line_ex(rl.Vector2(a[0], a[1]), rl.Vector2(b[0], b[1]), thickness, color)


def draw_reach_circle(base, r_max):
    o = to_screen(base)
    rl.draw_circle_lines(int(o[0]), int(o[1]), r_max, rl.LIGHTGRAY)


def draw_arm(points, highlight=None):
    sp = [to_screen(p) for p in points]
    for a, b in zip(sp, sp[1:]):
        _line(a, b, 12, rl.DARKGREEN)
    for i, p in enumerate(sp[:-1]):
        rl.draw_circle(int(p[0]), int(p[1]), 9, rl.MAROON)
        if i == highlight:                               # sendi yang sedang diproses (mode langkah)
            rl.draw_circle_lines(int(p[0]), int(p[1]), 15, rl.ORANGE)
            rl.draw_circle_lines(int(p[0]), int(p[1]), 16, rl.ORANGE)
    rl.draw_circle(int(sp[-1][0]), int(sp[-1][1]), 7, rl.SKYBLUE)


def draw_target(target, reachable):
    t = to_screen(target)
    rl.draw_circle_lines(int(t[0]), int(t[1]), 10, rl.RED if reachable else rl.ORANGE)


def draw_step_vectors(pivot, end_effector, target):
    """Vektor a (pivot -> end effector, biru) dan b (pivot -> target, merah) pada sendi aktif."""
    p, e, t = to_screen(pivot), to_screen(end_effector), to_screen(target)
    _line(p, t, 3, rl.RED)
    _line(p, e, 3, rl.BLUE)
    rl.draw_text("b", int((p[0] + t[0]) / 2) + 6, int((p[1] + t[1]) / 2) - 6, 18, rl.RED)
    rl.draw_text("a", int((p[0] + e[0]) / 2) + 6, int((p[1] + e[1]) / 2) - 6, 18, rl.BLUE)


def draw_convergence(history, max_iter):
    """Grafik batang jarak per iterasi pada frame ini (skala log, seperti slide P11)."""
    x0, y_bottom, w, h = 560, 570, 220, 100
    rl.draw_rectangle_lines(x0, y_bottom - h, w, h, rl.GRAY)
    rl.draw_text("Jarak per iterasi (log)", x0, y_bottom - h - 20, 16, rl.DARKGRAY)
    bar_w = w // max_iter
    for i, d in enumerate(history):
        frac = (math.log10(max(d, 1e-3)) + 3) / 6        # 1e-3 -> 0, 1e3 -> 1
        bh = int(max(0.0, min(1.0, frac)) * h)
        rl.draw_rectangle(x0 + i * bar_w + 2, y_bottom - bh, bar_w - 4, bh, rl.ORANGE)


def draw_text_lines(lines, x=20, y=15, size=18):
    """lines: daftar (teks, warna)."""
    for k, (text, color) in enumerate(lines):
        rl.draw_text(text, x, y + k * (size + 6), size, color)
