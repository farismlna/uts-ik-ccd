"""
Jalankan:  pip install raylib   lalu   python main.py
Uji:       python test_ik.py 

Kontrol:
  Mouse  : posisi target
  TAB    : ganti mode OTOMATIS <-> LANGKAH (satu sendi per langkah)
  SPACE  : (mode langkah) terapkan satu langkah CCD pada sendi aktif
  L      : batas sendi hidup/mati        G : grafik konvergensi        R : reset
"""
import math
from dataclasses import dataclass, field

import pyray as rl

import config as cfg
import render
from kinematics import fk, reach_range
from ccd_solver import (ccd, ccd_joint_step, joint_vectors, joint_delta,
                        distance_to_target)

N = len(cfg.LENGTHS)


@dataclass
class State:
    angles: list = field(default_factory=lambda: list(cfg.START_ANGLES))
    use_limits: bool = False
    show_graph: bool = True
    step_mode: bool = False
    step_joint: int = N - 1          # sendi yang akan diproses berikutnya (terluar dulu)
    step_iter: int = 1               # iterasi ke berapa pada mode langkah
    history: list = field(default_factory=list)


def reset_steps(s):
    s.step_joint, s.step_iter = N - 1, 1


def handle_keys(s):
    if rl.is_key_pressed(rl.KEY_L):
        s.use_limits = not s.use_limits
    if rl.is_key_pressed(rl.KEY_G):
        s.show_graph = not s.show_graph
    if rl.is_key_pressed(rl.KEY_TAB):
        s.step_mode = not s.step_mode
        reset_steps(s)
    if rl.is_key_pressed(rl.KEY_R):
        s.angles[:] = cfg.START_ANGLES
        reset_steps(s)


def advance_step(s, target, limits):
    """Terapkan satu langkah CCD pada sendi aktif, lalu maju ke sendi berikutnya ke arah pangkal."""
    ccd_joint_step(cfg.BASE, cfg.LENGTHS, s.angles, target, s.step_joint, limits)
    if s.step_joint == 0:
        s.step_joint, s.step_iter = N - 1, s.step_iter + 1
    else:
        s.step_joint -= 1


def main():
    s = State()
    r_min, r_max = reach_range(cfg.LENGTHS)

    rl.init_window(cfg.WIN_W, cfg.WIN_H, "UTS - Inverse Kinematics (CCD, 3 segmen)")
    rl.set_target_fps(60)
    while not rl.window_should_close():
        handle_keys(s)
        m = rl.get_mouse_position()
        target = render.to_world(m.x, m.y)
        d_target = math.hypot(target[0] - cfg.BASE[0], target[1] - cfg.BASE[1])
        reachable = r_min <= d_target <= r_max
        limits = cfg.LIMITS_ON if s.use_limits else None

        if s.step_mode:
            if rl.is_key_pressed(rl.KEY_SPACE):
                advance_step(s, target, limits)
        else:   # warm start: s.angles dari frame sebelumnya jadi pose awal
            it, _, s.history = ccd(cfg.BASE, cfg.LENGTHS, s.angles, target,
                                   max_iter=cfg.MAX_ITER_PER_FRAME, tol=cfg.TOL, limits=limits)

        points = fk(cfg.BASE, cfg.LENGTHS, s.angles)
        dist = distance_to_target(cfg.BASE, cfg.LENGTHS, s.angles, target)

        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)
        render.draw_reach_circle(cfg.BASE, r_max)
        render.draw_arm(points, highlight=s.step_joint if s.step_mode else None)
        render.draw_target(target, reachable)

        lines = []
        if s.step_mode:
            pivot, ee = joint_vectors(cfg.BASE, cfg.LENGTHS, s.angles, target, s.step_joint)
            delta = joint_delta(pivot, ee, target)
            render.draw_step_vectors(pivot, ee, target)
            lines.append((f"MODE LANGKAH  iterasi {s.step_iter}, sendi {s.step_joint + 1}   "
                          f"delta = {math.degrees(delta):.1f} derajat", rl.DARKBLUE))
            lines.append(("SPACE: terapkan langkah   TAB: kembali ke otomatis", rl.GRAY))
        else:
            lines.append((f"MODE OTOMATIS  iterasi: {len(s.history)}/{cfg.MAX_ITER_PER_FRAME}", rl.DARKBLUE))
        lines.append((f"Jarak ke target: {dist:.3f}", rl.DARKGRAY))
        ex, ey = points[-1]
        lines.append((f"Target: ({target[0]:.0f}, {target[1]:.0f})   End effector: ({ex:.0f}, {ey:.0f})",
                      rl.DARKGRAY))
        lines.append(("  ".join(f"theta{i + 1}={math.degrees(a):.0f}" for i, a in enumerate(s.angles)),
                      rl.DARKGRAY))
        lines.append(("Target dalam jangkauan" if reachable
                      else "Target di luar jangkauan: lengan meregang",
                      rl.DARKGREEN if reachable else rl.MAROON))
        render.draw_text_lines(lines)
        rl.draw_text(f"Batas sendi: {'ON' if s.use_limits else 'OFF'}   "
                     f"(TAB: mode | L: batas | G: grafik | R: reset)",
                     20, cfg.WIN_H - 28, 16, rl.GRAY)
        if s.show_graph and not s.step_mode:
            render.draw_convergence(s.history, cfg.MAX_ITER_PER_FRAME)
        rl.end_drawing()
    rl.close_window()


if __name__ == "__main__":
    main()
