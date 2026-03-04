"""


YOUR MISSION
─────────────
A drone is descending automatically toward the ground.
A landing platform oscillates sinusoidally (SHM) — it never stops moving.
You must write an autonomous controller that:

  PHASE 1 — SEARCH:
    The drone starts near the top-left of the arena, far from the platform.
    Design a search pattern to sweep the arena until the ArUco marker on
    the platform enters the camera's field of view.

  PHASE 2 — TRACK & LAND:
    Once detected, use a PID controller to keep the drone aligned with
    the moving platform as it descends.
    Success = final horizontal error <= 5 cm at touchdown.

HOW IT WORKS
─────────────
  This solver directly imports the simulator environment from simulator.py.
  Each frame you call sim.step_env(vx, vy) to send velocity commands and
  receive the camera image back. No files, no keyboard.

  Run with a SINGLE command:
      python solver.py

  The simulation window opens automatically.

ENVIRONMENT API  (this is your only interface — read carefully)
────────────────────────────────────────────────────────────────
  sim = make_sim()
        Creates the simulator. Do NOT touch the sim object directly.

  pixels, done = sim.step_env(vx, vy)
        Advances physics by one frame and returns:
          pixels  list[int]  Flat 10 000-element grayscale image (100x100).
                             Row-major order: index = row * 100 + col.
                             Values 0-255.
          done    bool       True when the drone has reached the ground.

        Your inputs:
          vx  float   Horizontal velocity command (m/s). Positive = RIGHT.
          vy  float   Vertical   velocity command (m/s). Positive = DOWN.
          Speed is capped at 5 m/s by the simulator.

  sim.drone_altitude  float   Current altitude in metres (10.0 -> 0.0).
  sim.fov_m           float   Camera ground footprint in metres at this altitude.
                              fov_m = 0.30 x altitude.
                              Use this to convert image offsets -> real metres.

CAMERA IMAGE FORMAT
────────────────────
  pixels is a flat list of 10 000 integers (0-255), row-major.

  What you will see:
    Grass / background  ->  dark gray    (~45-90)
    Platform surface    ->  bright gray  (~200-230)
    Inner ArUco square  ->  near black   (~0-20)  at the centre of the platform

  The platform appears as a bright rectangular cluster with a dark centre.
  It may be partially visible (clipped at the image edge) or absent entirely.

YOUR TASKS
───────────
  TODO 1 — Tune SEARCH_SPEED
  TODO 2 — Tune PID gains  (Kp, Ki, Kd)
  TODO 3 — Implement detect_platform()   [ArUco / bright-blob detection]
  TODO 4 — Implement PID.update()        [P + I + D terms]
  TODO 5 — Implement search_velocity()   [your search pattern]
  TODO 6 — Wire up the error -> PID -> velocity in main()

GRADING
────────
  Final error <= 0.05 m  (SUCCESS)        
  Bonus: error <= 0.02 m                     

RULES
──────
  OK   Only edit THIS file.
  OK   Tune any constant marked TODO.
  NO   Do NOT modify simulator_level2.py.
"""

import math
import sys

# ════════════════════════════════════════════════════════════════════
#  ENVIRONMENT BRIDGE
#  Imports the simulator and exposes a clean autonomous API.
#  ── DO NOT MODIFY THIS SECTION ──
# ════════════════════════════════════════════════════════════════════

try:
    import simulator_level2 as _sim_module
    import pygame
except ImportError as e:
    print(f"[ERROR] Could not import simulator: {e}")
    print("Ensure simulator.py is in the same directory.")
    sys.exit(1)


def make_sim():
    """Create the simulation environment. Keyboard control is disabled."""
    return _SimEnv()


class _SimEnv:
    """
    Wraps DroneSim in autonomous mode.
    Keyboard overrides are suppressed every frame — only your vx/vy commands
    control the drone.
    """

    CAM_RESOLUTION       = 100
    CAM_GROUND_SIZE_BASE = 0.30
    INITIAL_ALTITUDE     = 10.0
    SIM_TIME_SEC         = 35
    PIXELS_PER_METER     = 100
    FPS                  = 30

    def __init__(self):
        self._sim = _sim_module.DroneSim(mode=_sim_module.DroneSim.MODE_EXTERNAL)

    @property
    def drone_altitude(self):
        """Current altitude in metres."""
        return self._sim.drone_altitude

    @property
    def fov_m(self):
        """Camera ground footprint in metres (shrinks as drone descends)."""
        return self.CAM_GROUND_SIZE_BASE * max(0.1, self._sim.drone_altitude)

    def step_env(self, vx: float, vy: float):
        """
        Send a velocity command and advance the simulation by one frame.

        Returns:
            pixels  list[int]  Flat 100x100 grayscale image (10 000 values).
            done    bool       True when altitude reaches zero.
        """
        s  = self._sim
        dt = 1.0 / self.FPS

        # Keep pygame window responsive; ESC exits cleanly
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit(0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit(); sys.exit(0)

        # Suppress keyboard — autonomous only
        s._kb_vx = 0.0
        s._kb_vy = 0.0

        # Physics
        s.update_platform(dt)
        s.drone_altitude -= s.descent_rate * dt
        s.generate_camera_feed()

        spd = math.hypot(vx, vy)
        if spd > 5.0:
            vx *= 5.0 / spd
            vy *= 5.0 / spd

        s.drone_x += vx * self.PIXELS_PER_METER * dt
        s.drone_y += vy * self.PIXELS_PER_METER * dt
        s.drone_x  = max(20, min(s.W - 20, s.drone_x))
        s.drone_y  = max(20, min(s.H - 20, s.drone_y))

        err = math.hypot(s.drone_x - s.plat_x,
                         s.drone_y - s.plat_y) / self.PIXELS_PER_METER
        s.error_history.append(err)
        if len(s.error_history) > s.max_history:
            s.error_history.pop(0)

        s.trail.append((s.drone_x, s.drone_y))
        if len(s.trail) > s.trail_max_len:
            s.trail.pop(0)

        s.step += 1
        s._render()
        pygame.display.flip()
        s.clock.tick(self.FPS)

        # Convert camera surface -> flat pixel list
        pixels = []
        if s._last_cam_surface is not None:
            res = self.CAM_RESOLUTION
            for row in range(res):
                for col in range(res):
                    r, g, b, *_ = s._last_cam_surface.get_at((col, row))
                    pixels.append(int(0.299 * r + 0.587 * g + 0.114 * b))

        done = s.drone_altitude <= 0
        if done:
            final_err = math.hypot(s.drone_x - s.plat_x,
                                   s.drone_y - s.plat_y) / self.PIXELS_PER_METER
            success = final_err <= _sim_module.LANDING_SUCCESS_M
            s._render_result(success, final_err)
            pygame.display.flip()
            pygame.time.wait(4000)
            pygame.quit()

        return pixels, done


# ════════════════════════════════════════════════════════════════════
#  SECTION 1 — CONFIGURATION
#  TODO 1: Tune SEARCH_SPEED
#  TODO 2: Tune PID gains
# ════════════════════════════════════════════════════════════════════

SEARCH_SPEED = 1.5      # TODO 1: drone speed (m/s) during the search phase.
                        #   Faster = platform found sooner, but harder to brake
                        #   into accurate tracking. Try values between 1.0 - 4.0.

# Brightness cut-off for platform detection — pre-calibrated, do not change.
# Platform surface renders at ~200-230 gray; grass renders at ~45-90 gray.
BRIGHT_THRESHOLD = 150

# TODO 2: PID gains — X axis (horizontal)
KP_X = 0.0    # Proportional: how hard to correct current error
KI_X = 0.0    # Integral:     corrects persistent drift over time
KD_X = 0.0    # Derivative:   damps oscillation

# TODO 2: PID gains — Y axis (vertical / depth)
KP_Y = 0.0
KI_Y = 0.0
KD_Y = 0.0

OUTPUT_LIMIT = 4.0      # Max velocity the PID may output (m/s). Do not change.


# ════════════════════════════════════════════════════════════════════
#  SECTION 2 — PID CONTROLLER
#  TODO 4: Implement PID.update()
# ════════════════════════════════════════════════════════════════════

class PID:
    """
    Single-axis PID controller.

    Create one instance per axis:
        pid_x = PID(KP_X, KI_X, KD_X)
        pid_y = PID(KP_Y, KI_Y, KD_Y)

    Call every frame to get a velocity command:
        vx = pid_x.update(error_metres, dt)
    """

    def __init__(self, kp, ki, kd, limit=OUTPUT_LIMIT):
        self.kp    = kp
        self.ki    = ki
        self.kd    = kd
        self.limit = limit

        self._integral   = 0.0
        self._prev_error = 0.0

    def reset(self):
        """Zero internal state. Call once when the platform is first detected."""
        self._integral   = 0.0
        self._prev_error = 0.0

    def update(self, error, dt):
        """
        Compute the PID output for the given error and time-step.

        Args:
            error (float): Signed position error in metres.
                           Positive -> platform is to the RIGHT of / BELOW centre.
            dt    (float): Seconds since the last call (use 1/FPS = 0.033 s).

        Returns:
            float: Velocity command in m/s, clamped to +/- self.limit.

        TODO 4: Implement the three PID terms.
        ──────────────────────────────────────────────────────────────
          Proportional 
          Integral    
          Derivative   

          Return clamp(P + I + D, -self.limit, +self.limit).
        """
        if dt <= 0:
            return 0.0

        # ── your code here ──────────────────────────────────────────
        output = 0.0    # TODO 4: replace with P + I + D
        # ────────────────────────────────────────────────────────────

        return max(-self.limit, min(self.limit, output))


# ════════════════════════════════════════════════════════════════════
#  SECTION 3 — PLATFORM DETECTION
#  TODO 3: Implement detect_platform()
# ════════════════════════════════════════════════════════════════════

def detect_platform(pixels, resolution=100, threshold=BRIGHT_THRESHOLD):
    """
    Locate the landing platform in the camera image.

    The platform appears as a bright rectangular region (gray > threshold)
    with a small dark ArUco square at its centre.
    Return the normalised offset of the platform centre from the image centre.

    Args:
        pixels     list[int]   Flat 10 000-element grayscale image from step_env().
                               Pixel at (col, row):  pixels[row * resolution + col]
        resolution int         Image width = height = 100.
        threshold  int         Brightness cut-off (pre-set, do not change).

    Returns:
        (found, cx_norm, cy_norm)
          found    bool    True if platform detected.
          cx_norm  float   Horizontal offset from image centre.  Range [-1, +1].
                           -1 = far left,   0 = centre,   +1 = far right.
          cy_norm  float   Vertical offset from image centre.    Range [-1, +1].
                           -1 = top,        0 = centre,   +1 = bottom.

    TODO 3: Implement platform detection
    """
    # ── your code here ──────────────────────────────────────────────
    return False, 0.0, 0.0     
    # ────────────────────────────────────────────────────────────────


# ════════════════════════════════════════════════════════════════════
#  SECTION 4 — SEARCH STRATEGY
#  TODO 5: Implement search_velocity()
# ════════════════════════════════════════════════════════════════════

def search_velocity(search_timer, search_angle, search_radius, dt):
    """
    Compute drone velocity during the SEARCH phase.

    The drone starts near the top-left corner of the arena.
    The platform oscillates sinusoidally near the arena centre and never stops.
    You have approximately 35 seconds total before the drone touches down.

    Design a movement pattern that covers the arena efficiently so the
    platform enters the camera FOV as quickly as possible.

    Args:
        search_timer  float   Total seconds spent in search phase so far.
        search_angle  float   Persistent angle state (radians) — yours to use.
        search_radius float   Persistent radius state (metres) — yours to use.
        dt            float   Frame time in seconds (~0.033 at 30 FPS).

    Returns:
        (vx, vy, new_angle, new_radius)
          vx, vy        — velocity commands in m/s for this frame
          new_angle     — updated angle to be passed back in the next call
          new_radius    — updated radius to be passed back in the next call

    TODO 5: Replace the stub below with a real search pattern.
    ─────────────────────────────────────────────────────────────────
    Think about:
      - How much arena area does your pattern cover per second?
      - Does it pass through the centre where the platform spends most time?
      - Can it recover if the platform moves away before you reach it?
    """
    # ── your code here ──────────────────────────────────────────────
    # Stub: drone stays still. Replace with your search pattern.
    vx, vy     = 0.0, 0.0
    new_angle  = search_angle
    new_radius = search_radius
    return vx, vy, new_angle, new_radius
    # ────────────────────────────────────────────────────────────────


# ════════════════════════════════════════════════════════════════════
#  SECTION 5 — MAIN CONTROL LOOP
#  TODO 6: Wire up error -> PID -> velocity (marked inside)
# ════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("  Operation Touchdown — ART IITK")
    print("  Initialising simulation...")
    print("=" * 60)

    sim   = make_sim()
    pid_x = PID(KP_X, KI_X, KD_X)
    pid_y = PID(KP_Y, KI_Y, KD_Y)

    aruco_found   = False
    search_phase  = True
    search_angle  = 0.0
    search_radius = 0.0
    search_timer  = 0.0

    dt   = 1.0 / sim.FPS
    step = 0

    # First frame — hover to initialise the camera image
    pixels, done = sim.step_env(0.0, 0.0)

    while not done:

        # ── Detect platform in the current camera frame ────────────────────
        result = detect_platform(pixels)

        if result is None or len(result) < 3:
            # detect_platform not yet implemented — hover and wait
            pixels, done = sim.step_env(0.0, 0.0)
            step += 1
            continue

        found, cx_norm, cy_norm = result

        # ── Switch phases on first detection ──────────────────────────────
        if found and not aruco_found:
            print(f"  [DETECT] Platform acquired at step {step} "
                  f"({step / sim.FPS:.1f} s)  ->  PID tracking engaged.")
            aruco_found  = True
            search_phase = False
            pid_x.reset()
            pid_y.reset()

        # ── PHASE 1: Search ────────────────────────────────────────────────
        if search_phase:
            search_timer += dt
            vx, vy, search_angle, search_radius = search_velocity(
                search_timer, search_angle, search_radius, dt
            )
            if step % sim.FPS == 0:
                print(f"  [SEARCH]  step={step:4d}  t={search_timer:.1f}s  "
                      f"alt={sim.drone_altitude:.2f} m")

        # ── PHASE 2: PID Track & Land ──────────────────────────────────────
        elif aruco_found:
            #
            # TODO 6: Convert normalised image offsets to metres, then
            #         compute velocity commands through the PID controllers.
            # ──────────────────────────────────────────────────────────────
            #
            # cx_norm / cy_norm are in [-1, +1].
            # The camera covers sim.fov_m metres across its full width.
            # Therefore the half-width in metres is  sim.fov_m / 2.
            #
            # Real-world error:
            #     err_x_m = cx_norm * (sim.fov_m / 2)
            #     err_y_m = cy_norm * (sim.fov_m / 2)
            #
            # Velocity command from PID:
            #     vx = pid_x.update(err_x_m, dt)
            #     vy = pid_y.update(err_y_m, dt)
            #
            # ── your code here ──────────────────────────────────────
            err_x_m = 0.0   # TODO 6: replace
            err_y_m = 0.0   # TODO 6: replace
            vx      = 0.0   # TODO 6: replace with pid_x.update(err_x_m, dt)
            vy      = 0.0   # TODO 6: replace with pid_y.update(err_y_m, dt)
            # ────────────────────────────────────────────────────────

            if step % sim.FPS == 0:
                print(f"  [TRACK]   step={step:4d}  "
                      f"err=({err_x_m:+.3f}, {err_y_m:+.3f}) m  "
                      f"cmd=({vx:+.3f}, {vy:+.3f}) m/s  "
                      f"alt={sim.drone_altitude:.2f} m")

        # ── Platform temporarily lost — hover ──────────────────────────────
        else:
            vx, vy = 0.0, 0.0

        # ── Send command and receive next frame ────────────────────────────
        pixels, done = sim.step_env(vx, vy)
        step += 1

    print(f"\n  Simulation ended after {step} steps.")
    print("  Check the window for your final score.")


if __name__ == "__main__":
    main()