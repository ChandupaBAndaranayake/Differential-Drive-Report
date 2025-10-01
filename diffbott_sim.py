import pygame
import math
import sys
from collections import deque

# ---------- Simulation parameters ----------
SCREEN_W = 1000
SCREEN_H = 700
FPS = 60

# Robot geometry (arbitrary units; treat as meters)
WHEEL_RADIUS = 0.03  
WHEEL_BASE = 0.18     

# Robot visualization
ROBOT_LENGTH = 0.25
ROBOT_WIDTH = 0.18
TRAJECTORY_MAX = 10000

# ---------- Utility functions ----------
def wrap_to_pi(angle):
    a = ((angle + math.pi) % (2 * math.pi)) - math.pi
    return a

# ---------- Robot kinematics ----------
class DifferentialDriveRobot:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta  
        self.v = 0.0        
        self.omega = 0.0    
        self.left_wheel_omega = 0.0
        self.right_wheel_omega = 0.0

    def set_velocity(self, v, omega):
        self.v = v
        self.omega = omega

        R = WHEEL_RADIUS
        L = WHEEL_BASE
        
        self.left_wheel_omega = 0.0
        self.right_wheel_omega = 0.0

        if R > 0:
            self.right_wheel_omega = (2.0 * v + omega * L) / (2.0 * R)
            self.left_wheel_omega  = (2.0 * v - omega * L) / (2.0 * R)

    def step(self, dt):

        dx = self.v * math.cos(self.theta) * dt
        dy = self.v * math.sin(self.theta) * dt
        dtheta = self.omega * dt
        
        self.x += dx
        self.y += dy
        self.theta = wrap_to_pi(self.theta + dtheta)

# ---------- Controller ----------
class Controller:
    def __init__(self, mode='P_linear_P_angular'):
        """
        mode options:
         - 'P_linear_P_angular'
         - 'PD_linear_P_angular'
         - 'PD_linear_PID_angular'
        """
        self.mode = mode

        # Linear controller gains (for distance)
        self.kp_d = 0.9   # proportional gain for distance
        self.kd_d = 0.6   # derivative for distance (if PD)

        # Angular (heading) controller gains
        self.kp_th = 3.0
        self.ki_th = 0.2
        self.kd_th = 0.5

        # Internal states for derivative/integral
        self.prev_dist = None
        self.prev_heading_error = None
        self.int_heading = 0.0

        # Anti-windup limits
        self.int_max = 2.0

        # saturation
        self.v_max = 0.6   # m/s
        self.omega_max = 3.0  # rad/s

    def reset(self):
        self.prev_dist = None
        self.prev_heading_error = None
        self.int_heading = 0.0

    def compute_control(self, robot: DifferentialDriveRobot, target, dt):
        """
        Compute linear velocity v and angular velocity omega based on current robot state and target.

        Returns:
          (v, omega, debug_dict)
        """
        tx, ty = target
        dx = tx - robot.x
        dy = ty - robot.y
        dist = math.hypot(dx, dy)

        # desired heading (absolute)
        desired_theta = math.atan2(dy, dx)
        # heading error relative to robot
        heading_error = wrap_to_pi(desired_theta - robot.theta)

        # Compute derivative of distance (finite diff)
        dist_dot = 0.0
        if self.prev_dist is None:
            dist_dot = 0.0
        else:
            dist_dot = (dist - self.prev_dist) / max(dt, 1e-6)

        self.prev_dist = dist

        # Choose linear control law
        if self.mode == 'P_linear_P_angular':
            v = self.kp_d * dist
        elif self.mode == 'PD_linear_P_angular':
            v = self.kp_d * dist + self.kd_d * dist_dot
        elif self.mode == 'PD_linear_PID_angular':
            v = self.kp_d * dist + self.kd_d * dist_dot
        else:
            v = self.kp_d * dist

        # reduce forward speed when heading error large (helps avoid cutting corners)
        # simple gain: scale v by cos of heading error (no negative)
        heading_factor = max(0.0, math.cos(heading_error))
        v *= heading_factor

        # Angular control: simple P or PID depending on mode
        # heading derivative
        heading_dot = 0.0
        if self.prev_heading_error is None:
            heading_dot = 0.0
        else:
            heading_dot = (heading_error - self.prev_heading_error) / max(dt, 1e-6)
        self.prev_heading_error = heading_error

        if self.mode == 'PD_linear_P_angular':
            omega = self.kp_th * heading_error  
        elif self.mode == 'P_linear_P_angular':
            omega = self.kp_th * heading_error
        elif self.mode == 'PD_linear_PID_angular':
            # PID for heading
            self.int_heading += heading_error * dt
            # anti-windup
            self.int_heading = max(-self.int_max, min(self.int_max, self.int_heading))
            omega = (self.kp_th * heading_error
                     + self.ki_th * self.int_heading
                     + self.kd_th * heading_dot)
        else:
            omega = self.kp_th * heading_error

        # saturate
        v = max(-self.v_max, min(self.v_max, v))
        omega = max(-self.omega_max, min(self.omega_max, omega))

        debug = dict(dist=dist, heading_error=heading_error, dist_dot=dist_dot,
                     heading_dot=heading_dot, int_heading=self.int_heading,
                     v_unclamped=v, omega_unclamped=omega)
        return v, omega, debug

# ---------- Pygame display helpers ----------
def world_to_screen(x, y):
    """Map world coordinates (meters) to screen pixels."""
    # We center the world in the middle of the screen and flip y
    scale = 200.0  # 200 pix per meter (adjust to taste)
    cx = SCREEN_W // 2
    cy = SCREEN_H // 2
    sx = cx + int(x * scale)
    sy = cy - int(y * scale)
    return sx, sy

def draw_robot(surface, robot: DifferentialDriveRobot):
    sx, sy = world_to_screen(robot.x, robot.y)
    # triangle pointing in heading direction
    L = ROBOT_LENGTH * 200.0
    W = ROBOT_WIDTH * 200.0
    # base triangle in robot frame
    points = [
        ( L/2, 0.0),
        (-L/2,  W/2),
        (-L/2, -W/2),
    ]
    # rotate and translate
    cosr = math.cos(robot.theta)
    sinr = math.sin(robot.theta)
    pts = []
    for px, py in points:
        rx = px * cosr - py * sinr
        ry = px * sinr + py * cosr
        pts.append((sx + rx, sy - ry)) 
    pygame.draw.polygon(surface, (50, 150, 230), pts)
    # draw heading line
    hx = sx + cosr * L * 0.8
    hy = sy - sinr * L * 0.8
    pygame.draw.line(surface, (10, 10, 10), (sx, sy), (hx, hy), 2)

def draw_target(surface, target):
    tx, ty = target
    sx, sy = world_to_screen(tx, ty)
    pygame.draw.circle(surface, (200, 50, 50), (sx, sy), 8)
    # crosshair
    pygame.draw.line(surface, (200, 50, 50), (sx-12, sy), (sx+12, sy), 1)
    pygame.draw.line(surface, (200, 50, 50), (sx, sy-12), (sx, sy+12), 1)

def draw_text(surface, text, pos, size=18, color=(0,0,0)):
    font = pygame.font.SysFont("Arial", size)
    surf = font.render(text, True, color)
    surface.blit(surf, pos)

# ---------- Main ----------
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Differential Drive Robot - P/PD/PID simulation")
    clock = pygame.time.Clock()

    # initial robot at origin facing +x
    robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=0.0)
    controller = Controller(mode='P_linear_P_angular')
    controller.reset()

    # initial target
    target = (1.0, 0.5)  

    trajectory = deque(maxlen=TRAJECTORY_MAX)
    trajectory.append((robot.x, robot.y))

    paused = False
    auto_mode = True

    # stopping condition
    STOP_THRESH = 0.035
    arrived = False

    # print controls
    print("Controls:")
    print(" Left-click: set target")
    print(" 1: P linear + P angular")
    print(" 2: PD linear + P angular")
    print(" 3: PD linear + PID angular (recommended)")
    print(" r: reset robot")
    print(" space: toggle pause")
    print(" ESC: quit")

    #tuned gains
    controller.kp_d = 0.9
    controller.kd_d = 0.6
    controller.kp_th = 3.0
    controller.ki_th = 0.2
    controller.kd_th = 0.5

    # Main loop
    running = True
    while running:
        dt_ms = clock.tick(FPS)
        dt = dt_ms / 1000.0 

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                cx = SCREEN_W // 2
                cy = SCREEN_H // 2
                sx = (mx - cx) / 200.0
                sy = (cy - my) / 200.0
                target = (sx, sy)
                arrived = False
                controller.reset()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                    break
                elif event.key == pygame.K_1:
                    controller.mode = 'P_linear_P_angular'
                    controller.reset()
                    print("Controller: P_linear_P_angular")
                elif event.key == pygame.K_2:
                    controller.mode = 'PD_linear_P_angular'
                    controller.reset()
                    print("Controller: PD_linear_P_angular")
                elif event.key == pygame.K_3:
                    controller.mode = 'PD_linear_PID_angular'
                    controller.reset()
                    print("Controller: PD_linear_PID_angular")
                elif event.key == pygame.K_r:
                    robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=0.0)
                    trajectory.clear()
                    trajectory.append((robot.x, robot.y))
                    arrived = False
                    controller.reset()
                    print("Robot reset.")
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                    print("Paused:", paused)

        if not paused and not arrived:
            # compute control
            v_cmd, omega_cmd, debug = controller.compute_control(robot, target, dt)

            # set velocities on robot (could include actuator limits)
            robot.set_velocity(v_cmd, omega_cmd)
            robot.step(dt)

            trajectory.append((robot.x, robot.y))

            if debug['dist'] < STOP_THRESH:
                robot.set_velocity(0.0, 0.0)
                arrived = True
                print(f"Arrived at target within {STOP_THRESH} m.")

        # Drawing
        screen.fill((255, 255, 255))
        ox, oy = world_to_screen(0, 0)
        pygame.draw.line(screen, (30,30,30), (ox-15, oy), (ox+15, oy))
        pygame.draw.line(screen, (30,30,30), (ox, oy-15), (ox, oy+15))

        if len(trajectory) >= 2:
            pts = [world_to_screen(x,y) for x,y in trajectory]
            pygame.draw.lines(screen, (120, 200, 120), False, pts, 2)

        draw_target(screen, target)
        draw_robot(screen, robot)

        draw_text(screen, f"Controller mode: {controller.mode}", (10, 10))
        draw_text(screen, f"Robot pos: x={robot.x:.3f} m, y={robot.y:.3f} m, θ={robot.theta:.2f} rad", (10, 30))
        _, _, dbg = controller.compute_control(robot, target, max(dt, 1e-6))
        draw_text(screen, f"Distance to target: {dbg['dist']:.3f} m", (10, 50))
        draw_text(screen, f"Heading err: {dbg['heading_error']:.3f} rad", (10, 70))
        draw_text(screen, f"v_cmd: {robot.v:.3f} m/s, ω_cmd: {robot.omega:.3f} rad/s", (10, 90))
        draw_text(screen, "Left-click to place target. 1/2/3 change controllers. r reset. Space pause.", (10, SCREEN_H-30), size=16)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
