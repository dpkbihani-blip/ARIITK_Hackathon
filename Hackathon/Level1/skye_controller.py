import math
import pygame
from skye_env import SkyeEnv

TARGET_TRACK_DIST = 60
MAX_SPEED = 3.5
DANGER_DIST = 80
last_seen_target = None

def _lidar_repulsion(lidar_distances, num_rays=36, danger_dist=DANGER_DIST):
    """Returns (rx, ry) repulsion vector from LiDAR readings."""
    rx, ry = 0.0, 0.0

    if not lidar_distances:
        return rx, ry

    for i, dist in enumerate(lidar_distances):

        if dist < danger_dist and dist > 1e-6:

            angle = (i / num_rays) * 2 * math.pi

            strength = (danger_dist - dist) / danger_dist

            rx -= strength * math.cos(angle)
            ry -= strength * math.sin(angle)

    return rx, ry

def circular_exploration(lidar):
    """
    Move tangentially around closest obstacle.
    This makes the drone explore obstacle clusters,
    where SKYE-X usually hides.
    """

    if not lidar:
        return 0.5, 0.3

    closest = min(lidar)
    idx = lidar.index(closest)

    obstacle_angle = math.radians(idx * 10)

    # tangent direction
    tangent_angle = obstacle_angle + math.pi / 2

    vx = math.cos(tangent_angle)
    vy = math.sin(tangent_angle)

    rx, ry = _lidar_repulsion(lidar)

    vx += 1.5 * rx
    vy += 1.5 * ry

    return vx, vy


def pursue_target(px, py, tx, ty):

    dx = tx - px
    dy = ty - py

    dist = math.hypot(dx, dy)

    if dist < 1e-6:
        return 0, 0

    dx /= dist
    dy /= dist

    # Maintain distance near 60 px
    error = dist - TARGET_TRACK_DIST

    vx = dx * error
    vy = dy * error

    return vx, vy


def compute_velocity(sensors):

    global last_seen_target

    px = sensors["player_x"]
    py = sensors["player_y"]

    lidar = sensors.get("lidar_distances", [])

    rx, ry = _lidar_repulsion(lidar)

    vx, vy = 0.0, 0.0
    
    if sensors["target_visible"]:

        tx, ty = sensors["target_pos"]

        last_seen_target = (tx, ty)

        base_vx, base_vy = pursue_target(px, py, tx, ty)

    elif last_seen_target is not None:

        tx, ty = last_seen_target

        base_vx, base_vy = pursue_target(px, py, tx, ty)

    else:

        base_vx, base_vy = circular_exploration(lidar)

    vx = base_vx + rx
    vy = base_vy + ry

    speed = math.hypot(vx, vy)

    if speed > MAX_SPEED and speed > 1e-6:

        vx = (vx / speed) * MAX_SPEED
        vy = (vy / speed) * MAX_SPEED

    return vx, vy


# ==============================
# SIMULATION LOOP
# ==============================

def main():

    print("--- SKYE-X Booting: Search & Pursuit ---")

    env = SkyeEnv()

    running = True

    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        sensors = env.get_sensor_data()

        vx, vy = compute_velocity(sensors)

        env.step(vx, vy)

        if env.crashed:
            print(f"MISSION FAILED: Drone Crashed! Final Score: {env.score}")
            break

        elif env.mission_over:
            print(f"SIMULATION COMPLETE. Final Escort Score: {env.score} timesteps")
            break

    pygame.quit()


if __name__ == "__main__":
    main()
