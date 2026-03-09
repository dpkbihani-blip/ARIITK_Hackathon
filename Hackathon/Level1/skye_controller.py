import math
import pygame
import numpy as np
from skye_env import SkyeEnv


# ==============================================================
# CONFIGURATION
# ==============================================================

WIDTH = 1280
HEIGHT = 900

NUM_RAYS = 36
RAY_STEP = 10

MAX_SPEED = 3.5
SAFE_SPEED = 2.2

TARGET_TRACK_DISTANCE = 60

DANGER_DIST = 80
CRITICAL_DIST = 15

CELL = 20

GRID_W = WIDTH // CELL
GRID_H = HEIGHT // CELL


# ==============================================================
# GLOBAL STATE
# ==============================================================

occupancy_grid = np.zeros((GRID_W, GRID_H))

last_seen_target = None

trajectory_memory = np.array([1.0, 0.0])

corridor_mode = False
current_corridor_dir = np.array([0.0, 0.0])

# --- stuck detection memory ---
position_history = []
heading_history = []

HISTORY_SIZE = 30
STUCK_RADIUS = 25


# ==============================================================
# VECTOR UTILITIES
# ==============================================================

def normalize(vx,vy):

    s = math.hypot(vx,vy)

    if s < 1e-6:
        return 0,0

    return vx/s,vy/s


# ==============================================================
# MAP FUNCTIONS
# ==============================================================

def world_to_grid(x,y):

    gx = int(x // CELL)
    gy = int(y // CELL)

    gx = max(0,min(GRID_W-1,gx))
    gy = max(0,min(GRID_H-1,gy))

    return gx,gy


def grid_to_world(gx,gy):

    return gx*CELL + CELL/2 , gy*CELL + CELL/2


# ==============================================================
# LIDAR MAPPING
# ==============================================================

def update_map(px,py,lidar):

    for i,d in enumerate(lidar):

        angle = math.radians(i*RAY_STEP)

        steps = int(d/CELL)

        for step in range(steps):

            x = px + step*CELL*math.cos(angle)
            y = py + step*CELL*math.sin(angle)

            gx,gy = world_to_grid(x,y)

            occupancy_grid[gx,gy] = -1

        ox = px + d*math.cos(angle)
        oy = py + d*math.sin(angle)

        gx,gy = world_to_grid(ox,oy)

        occupancy_grid[gx,gy] = 1


# ==============================================================
# FRONTIER EXPLORATION
# ==============================================================

def detect_frontiers():

    frontiers=[]

    for x in range(1,GRID_W-1):

        for y in range(1,GRID_H-1):

            if occupancy_grid[x,y] < 0:

                for dx in [-1,0,1]:
                    for dy in [-1,0,1]:

                        nx=x+dx
                        ny=y+dy

                        if occupancy_grid[nx,ny]==0:

                            frontiers.append((x,y))
                            break

    return frontiers


def choose_frontier(px,py):

    frontiers=detect_frontiers()

    if not frontiers:
        return None

    best=None
    best_dist=1e9

    for cell in frontiers:

        wx,wy=grid_to_world(cell[0],cell[1])

        d=math.hypot(wx-px,wy-py)

        if d<best_dist:
            best_dist=d
            best=cell

    if best is None:
        return None

    return grid_to_world(best[0],best[1])


# ==============================================================
# CORRIDOR DETECTION
# ==============================================================

def detect_corridors(lidar):

    corridors=[]
    threshold=100
    min_gap=4

    current=[]

    for i,d in enumerate(lidar):

        if d>threshold:
            current.append(i)

        else:
            if len(current)>=min_gap:
                corridors.append(current.copy())
            current=[]

    if len(current)>=min_gap:
        corridors.append(current)

    return corridors


def corridor_direction(corridor):

    idx=corridor[len(corridor)//2]

    angle=math.radians(idx*10)

    return math.cos(angle),math.sin(angle)


def detect_dead_end(lidar):

    front=[0,1,35]

    d=min(lidar[i] for i in front)

    return d<30


# ==============================================================
# OBSTACLE AVOIDANCE
# ==============================================================

def lidar_repulsion(lidar):

    rx=0
    ry=0

    for i,d in enumerate(lidar):

        if d<DANGER_DIST:

            angle=(i/NUM_RAYS)*2*math.pi

            strength=(DANGER_DIST-d)/DANGER_DIST

            rx-=strength*math.cos(angle)
            ry-=strength*math.sin(angle)

    return rx,ry


def emergency_escape(lidar):

    idx=lidar.index(max(lidar))

    idx=max(0,min(NUM_RAYS-1,idx + np.random.randint(-3,4)))

    angle=math.radians(idx*10)

    return math.cos(angle),math.sin(angle)


# ==============================================================
# ADVANCED STUCK DETECTION
# ==============================================================

def detect_stuck(px,py,vx,vy):

    global position_history
    global heading_history

    position_history.append((px,py))
    heading_history.append((vx,vy))

    if len(position_history) > HISTORY_SIZE:
        position_history.pop(0)

    if len(heading_history) > HISTORY_SIZE:
        heading_history.pop(0)

    if len(position_history) < HISTORY_SIZE:
        return False

    xs=[p[0] for p in position_history]
    ys=[p[1] for p in position_history]

    span_x=max(xs)-min(xs)
    span_y=max(ys)-min(ys)

    radius=max(span_x,span_y)

    speed=math.hypot(vx,vy)

    if radius < STUCK_RADIUS:
        return True

    if speed < 0.1:
        return True

    # detect circular loops
    angles=[math.atan2(h[1],h[0]) for h in heading_history]

    if max(angles)-min(angles) > math.pi*1.8:
        return True

    return False


# ==============================================================
# PURSUIT CONTROLLER
# ==============================================================

def pursuit(px,py,tx,ty):

    dx=tx-px
    dy=ty-py

    dist=math.hypot(dx,dy)

    if dist<1e-6:
        return 0,0

    dx/=dist
    dy/=dist

    radial=dist-TARGET_TRACK_DISTANCE

    vx=dx*radial
    vy=dy*radial

    vx+=-0.7*dy
    vy+=0.7*dx

    return vx,vy


# ==============================================================
# NAVIGATION
# ==============================================================

def go_to(px,py,tx,ty):

    dx=tx-px
    dy=ty-py

    return normalize(dx,dy)


# ==============================================================
# MAIN CONTROLLER
# ==============================================================

def compute_velocity(sensors):

    global last_seen_target
    global trajectory_memory
    global corridor_mode
    global current_corridor_dir

    px=sensors["player_x"]
    py=sensors["player_y"]

    lidar=sensors["lidar_distances"]

    update_map(px,py,lidar)

    if sensors["target_visible"]:

        tx,ty=sensors["target_pos"]
        last_seen_target=(tx,ty)

    elif last_seen_target:

        tx,ty=last_seen_target

    else:

        tx,ty=None,None


    # ======================================================
    # PURSUIT MODE
    # ======================================================

    if tx is not None:

        vx,vy=pursuit(px,py,tx,ty)

        avoid=2.5


    # ======================================================
    # EXPLORATION MODE
    # ======================================================

    else:

        corridors=detect_corridors(lidar)

        if corridor_mode:

            if detect_dead_end(lidar):

                vx=-current_corridor_dir[0]
                vy=-current_corridor_dir[1]

                corridor_mode=False

            else:

                vx=current_corridor_dir[0]
                vy=current_corridor_dir[1]

        else:

            if corridors:

                best=max(corridors,key=len)

                vx,vy=corridor_direction(best)

                current_corridor_dir=np.array([vx,vy])

                corridor_mode=True

            else:

                frontier=choose_frontier(px,py)

                if frontier:

                    vx,vy=go_to(px,py,frontier[0],frontier[1])

                else:

                    vx,vy=trajectory_memory

        avoid=1.5


    # ======================================================
    # OBSTACLE AVOIDANCE
    # ======================================================

    rx,ry=lidar_repulsion(lidar)

    vx+=avoid*rx
    vy+=avoid*ry


    # ======================================================
    # EMERGENCY ESCAPE
    # ======================================================

    if min(lidar)<CRITICAL_DIST:

        vx,vy=emergency_escape(lidar)

        corridor_mode=False


    # ======================================================
    # STUCK ESCAPE
    # ======================================================

    if detect_stuck(px,py,vx,vy):

        vx,vy=emergency_escape(lidar)

        corridor_mode=False


    # ======================================================
    # SPEED CONTROL
    # ======================================================

    speed=math.hypot(vx,vy)

    if min(lidar)>120:
        target=MAX_SPEED
    else:
        target=SAFE_SPEED

    if speed>1e-6:

        vx=vx/speed*target
        vy=vy/speed*target


    trajectory_memory=np.array([vx,vy])

    return vx,vy


# ==============================================================
# SIMULATION LOOP
# ==============================================================

def main():

    env=SkyeEnv()

    running=True

    while running:

        for event in pygame.event.get():

            if event.type==pygame.QUIT:
                running=False

        sensors=env.get_sensor_data()

        vx,vy=compute_velocity(sensors)

        env.step(vx,vy)

        if env.crashed:

            print("MISSION FAILED:",env.score)
            break

        if env.mission_over:

            print("FINAL SCORE:",env.score)
            break

    pygame.quit()


if __name__=="__main__":

    main()
