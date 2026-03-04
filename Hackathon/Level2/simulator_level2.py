"""
Operation Touchdown Simulator - ART IITK  

Changes vs original:
  1. Top-left info box is smaller (compact single-line bar, no big block)
  2. Helipad oscillates with a small downward drift component
     (SHM in X + slow sinusoidal drift in Y)
  3. Keyboard interaction is FULLY ACTIVE in ALL modes (demo, keyboard, external)
     In --c (solver) mode: holding a key overrides the solver commands instantly.

Run modes:
  python simulator.py             → Demo  (keyboard active)
  python simulator.py --keyboard  → Keyboard only
  python simulator.py --c         → External solver  (keyboard still overrides)
"""
import pygame
import math
import os
import sys

# ── Constants ───────────────────────────────────────────────────────────────
PIXELS_PER_METER     = 100
FPS                  = 30
SIM_TIME_SEC         = 35
LANDING_SUCCESS_M    = 0.05

WHITE         = (255, 255, 255)
BLACK         = (0,   0,   0  )
GRAY          = (150, 150, 150)
DARK_GRAY     = (80,  80,  80 )
BG_COLOR      = (45,  90,  50 )
PLATFORM_COLOR= (210, 210, 210)
DRONE_COLOR   = (255, 60,  60 )
TRAIL_COLOR   = (255, 200, 100)
SUCCESS_COLOR = (80,  255, 120)
FAIL_COLOR    = (255, 80,  80 )
ARUCO_COLOR   = (255, 215, 0  )
ACCENT        = (100, 180, 255)
PANEL_BG      = (18,  22,  30 )


# ── Simulator ───────────────────────────────────────────────────────────────
class DroneSim:
    MODE_DEMO     = "demo"
    MODE_KEYBOARD = "keyboard"
    MODE_EXTERNAL = "external"

    def __init__(self, mode=MODE_DEMO):
        pygame.init()
        self.W, self.H = 1100, 820
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("ART IITK — Operation Touchdown  |  ESC to quit")

        self.mode = mode
        self.time_elapsed = 0.0

        # Platform SHM (X) + small Y oscillation
        self.plat_size   = 1.0 * PIXELS_PER_METER
        self.start_x     = self.W / 2
        self.start_y     = self.H / 2
        self.plat_x      = self.start_x / 2
        self.plat_y      = self.start_y
        self.amplitude   = 2.0 * PIXELS_PER_METER
        self.omega       = 0.9
        self.amp_y       = 0.6 * PIXELS_PER_METER
        self.omega_y     = 0.45

        # Drone
        self.drone_x        = 80.0
        self.drone_y        = 180.0
        self.drone_altitude = 10.0
        self.descent_rate   = self.drone_altitude / SIM_TIME_SEC

        # Camera
        self.cam_resolution       = 100
        self.cam_ground_size_base = 0.30
        self._last_cam_surface    = None

        # Detection tracking
        self.aruco_visible  = False
        self.detection_step = None

        # Error history for live graph
        self.error_history = []
        self.max_history   = 200

        # Trail
        self.trail         = []
        self.trail_max_len = 200

        # Keyboard velocities — always tracked in all modes
        self._kb_vx = 0.0
        self._kb_vy = 0.0

        # Fonts
        self.font_sm   = pygame.font.SysFont("consolas", 17, bold=False)
        self.font_med  = pygame.font.SysFont("consolas", 20, bold=True)
        self.font_big  = pygame.font.SysFont("consolas", 42, bold=True)
        self.font_tiny = pygame.font.SysFont("consolas", 14, bold=False)

        self.clock = pygame.time.Clock()
        self.step  = 0

    # ── Platform ──────────────────────────────────────────────────────────
    def update_platform(self, dt):
        self.time_elapsed += dt
        self.plat_x = self.start_x + self.amplitude * math.sin(self.omega * self.time_elapsed)
        self.plat_y = self.start_y + self.amp_y * math.sin(self.omega_y * self.time_elapsed)

    # ── Camera Feed ───────────────────────────────────────────────────────
    def generate_camera_feed(self):
        ground_m  = self.cam_ground_size_base * max(0.1, self.drone_altitude)
        ground_px = ground_m * PIXELS_PER_METER
        cam_size  = int(min(800, max(100, ground_px)))

        surf = pygame.Surface((cam_size, cam_size))
        surf.fill(BG_COLOR)

        rel_x = self.plat_x - (self.drone_x - cam_size / 2)
        rel_y = self.plat_y - (self.drone_y - cam_size / 2)

        r = pygame.Rect(rel_x - self.plat_size/2, rel_y - self.plat_size/2,
                        self.plat_size, self.plat_size)
        pygame.draw.rect(surf, PLATFORM_COLOR, r)
        pygame.draw.rect(surf, ARUCO_COLOR, r, 4)

        inner = pygame.Rect(rel_x - self.plat_size/4, rel_y - self.plat_size/4,
                            self.plat_size/2, self.plat_size/2)
        pygame.draw.rect(surf, BLACK, inner)

        scaled = pygame.transform.scale(surf, (self.cam_resolution, self.cam_resolution))
        self._last_cam_surface = scaled

        bright = sum(
            1 for y in range(self.cam_resolution)
            for x in range(self.cam_resolution)
            if scaled.get_at((x, y))[0] > 150
        )
        self.aruco_visible = bright > 4
        if self.aruco_visible and self.detection_step is None:
            self.detection_step = self.step

        try:
            with open("camera_pixels.txt", "w") as f:
                for y in range(self.cam_resolution):
                    for x in range(self.cam_resolution):
                        r2, g, b, *_ = scaled.get_at((x, y))
                        gray = int(0.299*r2 + 0.587*g + 0.114*b)
                        f.write(f"{gray} ")
                    f.write("\n")
        except IOError:
            pass

    # ── Commands ──────────────────────────────────────────────────────────
    def read_commands(self):
        vx, vy = 0.0, 0.0
        try:
            if os.path.exists("commands.txt"):
                with open("commands.txt", "r") as f:
                    data = f.read().strip().split()
                    if len(data) >= 2:
                        vx, vy = float(data[0]), float(data[1])
        except Exception:
            pass
        return vx, vy

    # ── Main Loop ─────────────────────────────────────────────────────────
    def run(self):
        running = True
        KB_SPD  = 3.5   # keyboard speed m/s

        while running and self.drone_altitude > 0:
            dt = self.clock.tick(FPS) / 1000.0

            # ── Event pump ────────────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

            # ── Keyboard state — ALWAYS polled in every mode ──────────────
            keys = pygame.key.get_pressed()
            self._kb_vx = (
                -KB_SPD if (keys[pygame.K_LEFT]  or keys[pygame.K_a]) else
                 KB_SPD if (keys[pygame.K_RIGHT] or keys[pygame.K_d]) else 0.0
            )
            self._kb_vy = (
                -KB_SPD if (keys[pygame.K_UP]   or keys[pygame.K_w]) else
                 KB_SPD if (keys[pygame.K_DOWN] or keys[pygame.K_s]) else 0.0
            )

            self.update_platform(dt)
            self.drone_altitude -= self.descent_rate * dt
            self.generate_camera_feed()

            # ── Velocity arbitration ──────────────────────────────────────
            # EXTERNAL mode: keyboard takes priority when any key is held,
            #                otherwise fall back to solver's commands.txt
            # KEYBOARD/DEMO: keyboard always drives
            kb_pressed = (self._kb_vx != 0.0 or self._kb_vy != 0.0)

            if self.mode == self.MODE_EXTERNAL and not kb_pressed:
                vx, vy = self.read_commands()
            else:
                vx, vy = self._kb_vx, self._kb_vy

            # Speed cap
            spd = math.hypot(vx, vy)
            if spd > 5.0:
                vx *= 5.0 / spd
                vy *= 5.0 / spd

            self.drone_x += vx * PIXELS_PER_METER * dt
            self.drone_y += vy * PIXELS_PER_METER * dt
            self.drone_x  = max(20, min(self.W - 20, self.drone_x))
            self.drone_y  = max(20, min(self.H - 20, self.drone_y))

            err = math.hypot(self.drone_x - self.plat_x,
                             self.drone_y - self.plat_y) / PIXELS_PER_METER
            self.error_history.append(err)
            if len(self.error_history) > self.max_history:
                self.error_history.pop(0)

            self.trail.append((self.drone_x, self.drone_y))
            if len(self.trail) > self.trail_max_len:
                self.trail.pop(0)

            self.step += 1
            self._render()
            pygame.display.flip()

        final_err = math.hypot(self.drone_x - self.plat_x,
                               self.drone_y - self.plat_y) / PIXELS_PER_METER
        success   = final_err <= LANDING_SUCCESS_M
        self._render_result(success, final_err)
        pygame.display.flip()
        pygame.time.wait(4000)
        pygame.quit()

    # ── Drawing ───────────────────────────────────────────────────────────
    def _draw_grid(self, arena_rect):
        x0, y0, aw, ah = arena_rect
        step = PIXELS_PER_METER // 2
        for x in range(x0, x0 + aw + 1, step):
            c = (55, 75, 55) if x % PIXELS_PER_METER else (70, 95, 70)
            pygame.draw.line(self.screen, c, (x, y0), (x, y0 + ah), 1)
        for y in range(y0, y0 + ah + 1, step):
            c = (55, 75, 55) if y % PIXELS_PER_METER else (70, 95, 70)
            pygame.draw.line(self.screen, c, (x0, y), (x0 + aw, y), 1)

    def _draw_platform(self):
        px, py = int(self.plat_x), int(self.plat_y)
        hs = int(self.plat_size / 2)

        pygame.draw.ellipse(self.screen, (20, 40, 20),
                            (px - hs, py + hs - 6, hs*2, 12))
        r = pygame.Rect(px - hs, py - hs, hs*2, hs*2)
        pygame.draw.rect(self.screen, PLATFORM_COLOR, r, border_radius=4)
        pygame.draw.rect(self.screen, ARUCO_COLOR, r, 4, border_radius=4)
        pygame.draw.rect(self.screen, WHITE, r, 1, border_radius=4)

        inner = pygame.Rect(px - hs//2, py - hs//2, hs, hs)
        pygame.draw.rect(self.screen, (10, 10, 10), inner)
        pygame.draw.line(self.screen, ARUCO_COLOR, (px - hs//2, py), (px + hs//2, py), 1)
        pygame.draw.line(self.screen, ARUCO_COLOR, (px, py - hs//2), (px, py + hs//2), 1)

        sr = int(LANDING_SUCCESS_M * PIXELS_PER_METER)
        zone = pygame.Surface((sr*2+2, sr*2+2), pygame.SRCALPHA)
        pygame.draw.circle(zone, (*SUCCESS_COLOR, 50), (sr+1, sr+1), sr)
        pygame.draw.circle(zone, (*SUCCESS_COLOR, 200), (sr+1, sr+1), sr, 1)
        self.screen.blit(zone, (px - sr - 1, py - sr - 1))

    def _draw_drone(self):
        ix, iy = int(self.drone_x), int(self.drone_y)
        arm    = 26

        soff   = int(self.drone_altitude * 2.5)
        srad   = max(8, int(38 - self.drone_altitude * 2))
        salpha = max(40, int(190 - self.drone_altitude * 10))
        shadow = pygame.Surface((160, 160), pygame.SRCALPHA)
        pygame.draw.circle(shadow, (0, 0, 0, salpha),
                           (80 + soff, 80 + soff), srad)
        self.screen.blit(shadow, (ix - 80, iy - 80))

        for dx, dy in [(-1,-1),(1,1),(-1,1),(1,-1)]:
            pygame.draw.line(self.screen, (220,225,230),
                             (ix, iy), (ix + dx*arm, iy + dy*arm), 3)
        rr = 13 + int(math.sin(self.time_elapsed * 30) * 1)
        for dx, dy in [(-1,-1),(1,1),(-1,1),(1,-1)]:
            rx, ry = ix + dx*arm, iy + dy*arm
            pygame.draw.circle(self.screen, DARK_GRAY, (rx, ry), rr, 2)
            pygame.draw.circle(self.screen, (130,135,140), (rx, ry), rr - 3, 1)

        pygame.draw.circle(self.screen, DRONE_COLOR, (ix, iy), 11)
        pygame.draw.circle(self.screen, (255, 120, 120), (ix, iy), 6)
        pygame.draw.circle(self.screen, WHITE, (ix, iy), 11, 2)

        fov_m = self.cam_ground_size_base * max(0.1, self.drone_altitude)
        vr    = int(fov_m * PIXELS_PER_METER / 2)
        fov_s = pygame.Surface((vr*2+4, vr*2+4), pygame.SRCALPHA)
        pygame.draw.circle(fov_s, (*ARUCO_COLOR, 25), (vr+2, vr+2), vr)
        pygame.draw.circle(fov_s, (*ARUCO_COLOR, 130), (vr+2, vr+2), vr, 1)
        self.screen.blit(fov_s, (ix - vr - 2, iy - vr - 2))

    def _draw_trail(self):
        if len(self.trail) >= 2:
            for i in range(1, len(self.trail)):
                pygame.draw.line(self.screen, TRAIL_COLOR,
                                 (int(self.trail[i-1][0]), int(self.trail[i-1][1])),
                                 (int(self.trail[i][0]),   int(self.trail[i][1])), 2)

    def _draw_right_panel(self):
        pw  = 268
        px  = self.W - pw - 4
        pad = 10
        y   = 12

        panel = pygame.Surface((pw, self.H - 16), pygame.SRCALPHA)
        panel.fill((*PANEL_BG, 235))
        self.screen.blit(panel, (px, 8))

        cx = px + pw // 2

        # Title
        t = self.font_med.render("TELEMETRY", True, ACCENT)
        self.screen.blit(t, t.get_rect(centerx=cx, top=y)); y += 30
        pygame.draw.line(self.screen, (50,55,65), (px+8, y), (px+pw-8, y), 1); y += 8

        # Mode badge
        mode_labels = {
            self.MODE_DEMO:     ("DEMO",     GRAY),
            self.MODE_KEYBOARD: ("KEYBOARD", ACCENT),
            self.MODE_EXTERNAL: ("SOLVER",   (180, 255, 180)),
        }
        ml, mc = mode_labels[self.mode]
        badge = pygame.Surface((pw - 20, 26), pygame.SRCALPHA)
        badge.fill((*mc, 35))
        self.screen.blit(badge, (px + 10, y))
        pygame.draw.rect(self.screen, mc, (px+10, y, pw-20, 26), 1, border_radius=3)
        bt = self.font_med.render(f"MODE: {ml}", True, mc)
        self.screen.blit(bt, bt.get_rect(centerx=cx, top=y+4)); y += 36

        # Phase
        err_now = math.hypot(self.drone_x - self.plat_x,
                             self.drone_y - self.plat_y) / PIXELS_PER_METER
        if self.mode in (self.MODE_KEYBOARD, self.MODE_DEMO):
            phase_str, phase_col = "MANUAL FLIGHT", ACCENT
        elif not self.aruco_visible:
            phase_str, phase_col = "PHASE 1: SEARCH", (255, 200, 60)
        else:
            phase_str, phase_col = "PHASE 2: TRACK",  SUCCESS_COLOR

        ph = self.font_med.render(phase_str, True, phase_col)
        self.screen.blit(ph, ph.get_rect(centerx=cx, top=y)); y += 30
        pygame.draw.line(self.screen, (50,55,65), (px+8, y), (px+pw-8, y), 1); y += 8

        # Numeric rows
        err_col = SUCCESS_COLOR if err_now <= LANDING_SUCCESS_M else \
                  (255, 200, 60) if err_now < 0.20 else FAIL_COLOR
        aruco_str = "VISIBLE  ✓" if self.aruco_visible else "not in frame"
        aruco_col = SUCCESS_COLOR if self.aruco_visible else (180, 100, 100)

        rows = [
            ("Altitude",   f"{self.drone_altitude:.2f} m",  WHITE),
            ("H-Error",    f"{err_now:.3f} m",              err_col),
            ("Step",       f"{self.step}",                  GRAY),
            ("Time left",  f"{max(0.0, SIM_TIME_SEC - self.time_elapsed):.1f} s", GRAY),
            ("ArUco",      aruco_str,                       aruco_col),
        ]
        if self.detection_step is not None:
            t_det = self.detection_step / FPS
            rows.append(("Det. @", f"step {self.detection_step}  ({t_det:.1f}s)", SUCCESS_COLOR))

        for label, val, col in rows:
            lbl   = self.font_sm.render(label + ":", True, GRAY)
            val_s = self.font_sm.render(val, True, col)
            self.screen.blit(lbl,  (px + pad, y))
            self.screen.blit(val_s,(px + pw - val_s.get_width() - pad, y))
            y += 22

        y += 6
        pygame.draw.line(self.screen, (50,55,65), (px+8, y), (px+pw-8, y), 1); y += 10

        # Error graph
        gh = 70
        gw = pw - 20
        gl = self.font_sm.render("Error (m)   target: 0.05 m", True, GRAY)
        self.screen.blit(gl, (px + 10, y)); y += 18

        graph_rect = pygame.Rect(px + 10, y, gw, gh)
        pygame.draw.rect(self.screen, (10, 14, 20), graph_rect)
        pygame.draw.rect(self.screen, (50,55,65), graph_rect, 1)

        target_y_px = y + gh - int(0.05 / 0.5 * gh)
        pygame.draw.line(self.screen, (*SUCCESS_COLOR, 120),
                         (px+10, target_y_px), (px+10+gw, target_y_px), 1)

        if len(self.error_history) > 1:
            pts = []
            for i, e in enumerate(self.error_history):
                ex = px + 10 + int(i / self.max_history * gw)
                ey = y  + gh - int(min(e, 0.5) / 0.5 * gh)
                pts.append((ex, ey))
            pygame.draw.lines(self.screen, (100, 200, 255), False, pts, 2)

        y += gh + 8
        pygame.draw.line(self.screen, (50,55,65), (px+8, y), (px+pw-8, y), 1); y += 8

        # ── Keyboard widget — ALWAYS shown in every mode ──────────────────
        keys  = pygame.key.get_pressed()
        up    = bool(keys[pygame.K_UP]    or keys[pygame.K_w])
        down  = bool(keys[pygame.K_DOWN]  or keys[pygame.K_s])
        left  = bool(keys[pygame.K_LEFT]  or keys[pygame.K_a])
        right = bool(keys[pygame.K_RIGHT] or keys[pygame.K_d])

        kl = self.font_sm.render("KEYBOARD  (always active)", True, ACCENT)
        self.screen.blit(kl, kl.get_rect(centerx=cx, top=y)); y += 18

        ks  = 28
        gap = 4
        kx  = cx - ks - gap // 2

        def draw_key(label, active, kx2, ky2):
            col = ACCENT if active else (40, 45, 55)
            tc  = BLACK  if active else GRAY
            pygame.draw.rect(self.screen, col,
                             (kx2, ky2, ks, ks), border_radius=4)
            pygame.draw.rect(self.screen, (70,75,85),
                             (kx2, ky2, ks, ks), 1, border_radius=4)
            lt = self.font_sm.render(label, True, tc)
            self.screen.blit(lt, lt.get_rect(center=(kx2 + ks//2, ky2 + ks//2)))

        wx = cx - ks // 2
        draw_key("W", up,    wx,                  y)
        draw_key("A", left,  kx,                  y + ks + gap)
        draw_key("S", down,  kx + ks + gap,       y + ks + gap)
        draw_key("D", right, kx + ks*2 + gap*2,   y + ks + gap)

        y += ks * 2 + gap + 8
        pygame.draw.line(self.screen, (50,55,65), (px+8, y), (px+pw-8, y), 1); y += 8

        # Camera feed — sized to remaining panel height
        remaining = self.H - 16 - y - 22
        cam_size  = min(pw - 20, max(60, remaining - 20))

        cl = self.font_sm.render("DOWN-CAMERA FEED  (100x100)", True, GRAY)
        self.screen.blit(cl, (px + 10, y)); y += 18

        cam_rect = pygame.Rect(px + 10, y, cam_size, cam_size)
        pygame.draw.rect(self.screen, BLACK, cam_rect)
        if self._last_cam_surface:
            scaled = pygame.transform.scale(self._last_cam_surface, (cam_size, cam_size))
            self.screen.blit(scaled, (px + 10, y))
        pygame.draw.rect(self.screen, (50,55,65), cam_rect, 1)

        mid_x = px + 10 + cam_size // 2
        mid_y = y  + cam_size // 2
        pygame.draw.line(self.screen, (80,80,80),
                         (mid_x, y), (mid_x, y + cam_size), 1)
        pygame.draw.line(self.screen, (80,80,80),
                         (px+10, mid_y), (px+10+cam_size, mid_y), 1)
        y += cam_size + 6

        # Footer
        if self.mode == self.MODE_EXTERNAL:
            hint = "KB overrides solver when held"
            hcol = (255, 220, 100)
        else:
            hint = "WASD / Arrow keys to fly"
            hcol = ACCENT
        h = self.font_sm.render(hint, True, hcol)
        self.screen.blit(h, h.get_rect(centerx=cx, top=y))

    def _draw_top_bar(self):
        bar_w = self.W - 268 - 20
        bar_h = 22
        bx, by = 8, 8

        pygame.draw.rect(self.screen, (10,14,20), (bx, by, bar_w, bar_h), border_radius=4)
        frac   = min(1.0, self.time_elapsed / SIM_TIME_SEC)
        col    = SUCCESS_COLOR if frac < 0.7 else (255,200,60) if frac < 0.9 else FAIL_COLOR
        fill_w = max(4, int((bar_w - 4) * frac))
        pygame.draw.rect(self.screen, col,
                         (bx+2, by+2, fill_w, bar_h-4), border_radius=3)
        pygame.draw.rect(self.screen, (50,55,65), (bx, by, bar_w, bar_h), 1, border_radius=4)

        lc = BLACK if frac < 0.5 else PANEL_BG if frac < 0.9 else WHITE
        t  = self.font_tiny.render(
            f"  {self.time_elapsed:.1f} / {SIM_TIME_SEC} s"
            f"  |  alt {self.drone_altitude:.1f} m"
            f"  |  WASD/Arrows always active  |  ESC quit",
            True, lc
        )
        self.screen.blit(t, (bx + 4, by + 4))

    def _render(self):
        self.screen.fill((15, 18, 15))

        arena_w    = self.W - 268 - 16
        arena_rect = (8, 38, arena_w, self.H - 46)
        pygame.draw.rect(self.screen, BG_COLOR, arena_rect)
        self._draw_grid(arena_rect)

        self._draw_trail()
        self._draw_platform()
        self._draw_drone()
        self._draw_top_bar()
        self._draw_right_panel()

        pygame.draw.rect(self.screen, (50,55,65), arena_rect, 2, border_radius=2)

    def _render_result(self, success, final_err):
        self._render()
        ov = pygame.Surface((self.W, self.H), pygame.SRCALPHA)
        ov.fill((0, 0, 0, 190))
        self.screen.blit(ov, (0, 0))

        color = SUCCESS_COLOR if success else FAIL_COLOR
        msg   = "LANDING SUCCESS!" if success else "LANDING FAILED"
        t1    = self.font_big.render(msg, True, color)
        self.screen.blit(t1, t1.get_rect(center=(self.W//2, self.H//2 - 70)))

        det_str = (f"ArUco detected at step {self.detection_step}  "
                   f"({self.detection_step/FPS:.1f} s)") \
                  if self.detection_step else "ArUco was never detected"

        for i, line in enumerate([
            f"Final error :  {final_err:.4f} m    (target <= {LANDING_SUCCESS_M} m)",
            det_str,
            f"Total steps :  {self.step}   ({self.time_elapsed:.1f} s)",
        ]):
            s = self.font_med.render(line, True, WHITE)
            self.screen.blit(s, s.get_rect(center=(self.W//2, self.H//2 + 5 + i*36)))

        sub = self.font_sm.render("Window closes in 4 seconds…", True, GRAY)
        self.screen.blit(sub, sub.get_rect(center=(self.W//2, self.H//2 + 140)))


# ── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    args = sys.argv[1:]
    if "--keyboard" in args:
        mode = DroneSim.MODE_KEYBOARD
    elif "--c" in args:
        mode = DroneSim.MODE_EXTERNAL
    else:
        mode = DroneSim.MODE_DEMO

    sim = DroneSim(mode=mode)
    sim.run()