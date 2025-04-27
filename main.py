# Import required libraries
import numpy as np
import control
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import matplotlib.animation as animation
import pygame

import matplotlib.pyplot as plt
# Define system parameters
g = 9.8    # gravity (m/s^2)
M = 0.5    # mass of the cart (kg)
m = 0.2    # mass of the pendulum (kg)
l = 0.3    # length to pendulum center of mass (m)

print("System parameters set!")

# Define the state-space A and B matrices
A = np.array([
    [0, 1, 0, 0],
    [0, 0, (m * g) / M, 0],
    [0, 0, 0, 1],
    [0, 0, (M + m) * g / (M * l), 0]
])

B = np.array([
    [0],
    [1 / M],
    [0],
    [1 / (M * l)]
])

print("State-space matrices A and B created!")

# Define LQR weight matrices
Q = np.diag([10, 1, 100, 1])  # Weight states: [x, ẋ, θ, θ̇]
R = np.array([[0.001]])       # Penalize large control inputs

# Solve for the LQR gain K
K, S, E = control.lqr(A, B, Q, R)

print("LQR gain matrix K:")
print(K)

# Define the closed-loop system dynamics
def closed_loop_dynamics(t, x):
    u = -K @ x  # Control input: u = -Kx
    dxdt = A @ x + B.flatten() * u  # System dynamics
    return dxdt

# Set initial condition: small displacement
x0 = [0.1, 0, np.deg2rad(10), 0]  # [position, velocity, angle (rad), angular velocity]

# Simulate
t_span = (0, 5)  # Simulate from t=0 to t=5 seconds
t_eval = np.linspace(t_span[0], t_span[1], 500)  # Points at which to save results

solution = solve_ivp(closed_loop_dynamics, t_span, x0, t_eval=t_eval)

# Plot the results
plt.figure(figsize=(12,8))
plt.subplot(2,1,1)
plt.plot(solution.t, solution.y[0], label='Cart Position (m)')
plt.plot(solution.t, np.rad2deg(solution.y[2]), label='Pendulum Angle (deg)')
plt.grid()
plt.legend()
plt.title('Self-Balancing Robot: State Response')

plt.subplot(2,1,2)
u = -K @ solution.y
plt.plot(solution.t, u.flatten(), label='Control Input (Force)')
plt.grid()
plt.legend()
plt.xlabel('Time (s)')
plt.title('Control Effort')

plt.tight_layout()
plt.show()

# Setup figure
fig, ax = plt.subplots(figsize=(8,4))
ax.set_xlim(-1, 1)
ax.set_ylim(-0.5, 1.5)
ax.set_xlabel('Cart Position (m)')
ax.set_ylabel('Height (m)')
ax.set_title('Self-Balancing Robot Animation')

# Cart
cart_width = 0.2
cart_height = 0.1
cart, = ax.plot([], [], 'k-', lw=5)

# Pendulum
pendulum, = ax.plot([], [], 'o-', lw=2)
import time



# Initialize pygame
pygame.init()

# Set up the display
width, height = 800, 400
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Self-Balancing Robot Live Animation')

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)

# Scale factors to convert meters -> pixels
pixels_per_meter = 300
cart_width_pixels = int(0.2 * pixels_per_meter)
cart_height_pixels = int(0.1 * pixels_per_meter)
pendulum_length_pixels = int(l * pixels_per_meter)

# External force
external_force = 0

# Closed-loop system with external force
def closed_loop_dynamics_keyboard(t, x):
    global external_force
    u = -K @ x + external_force
    dxdt = A @ x + B.flatten() * u
    return dxdt

# Set initial condition
x0 = [0.1, 0, np.deg2rad(10), 0]
x = np.array(x0)

# Timing
dt = 0.02  # 50 FPS
t = 0
running = True
clock = pygame.time.Clock()

while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        external_force = -2
    elif keys[pygame.K_RIGHT]:
        external_force = 2
    else:
        external_force = 0

    # Update dynamics
    dxdt = closed_loop_dynamics_keyboard(t, x)
    x = x + dxdt * dt
    t += dt

    # Clear screen
    screen.fill(WHITE)

    # Calculate positions
    # Calculate positions
if not np.isnan(x[0]):
    cart_x = int(width / 2 + x[0] * pixels_per_meter)
else:
    cart_x = int(width / 2)

cart_y = height // 2

if not np.isnan(x[2]):
    pendulum_end_x = int(cart_x + pendulum_length_pixels * np.sin(x[2]))
    pendulum_end_y = int(cart_y - pendulum_length_pixels * np.cos(x[2]))
else:
    pendulum_end_x = cart_x
    pendulum_end_y = cart_y - pendulum_length_pixels

    # Draw cart
    #pygame.draw.rect(screen, BLACK, (cart_x - cart_width_pixels//2, cart_y - cart_height_pixels//2, cart_width_pixels, cart_height_pixels))
    # Only draw if values are not NaN
if not np.isnan(cart_x) and not np.isnan(cart_y):
    pygame.draw.rect(
        screen,
        BLACK,
        (
            int(cart_x - cart_width_pixels // 2),
            int(cart_y - cart_height_pixels // 2),
            int(cart_width_pixels),
            int(cart_height_pixels)
        )
    )



    # Draw pendulum
    pygame.draw.line(screen, BLUE, (cart_x, cart_y), (pendulum_end_x, pendulum_end_y), 5)
    pygame.draw.circle(screen, BLUE, (pendulum_end_x, pendulum_end_y), 10)

    # Update display
    pygame.display.flip()

    # Control FPS
    clock.tick(1/dt)

pygame.quit()


# Convert logged data to numpy array
state_log = np.array(state_log)
time_log = state_log[:,0]
position_log = state_log[:,1]
velocity_log = state_log[:,2]
angle_log = state_log[:,3]
angular_velocity_log = state_log[:,4]
external_force_log = state_log[:,5]

# Plot the results
plt.figure(figsize=(12,8))
plt.subplot(3,1,1)
plt.plot(time_log, position_log, label='Cart Position (m)')
plt.plot(time_log, np.rad2deg(angle_log), label='Pendulum Angle (deg)')
plt.legend()
plt.grid()
plt.title('States with Keyboard Interaction')

plt.subplot(3,1,2)
plt.plot(time_log, external_force_log, label='User Applied Force')
plt.legend()
plt.grid()

plt.subplot(3,1,3)
u_log = -(K @ np.vstack((position_log, velocity_log, angle_log, angular_velocity_log)))
plt.plot(time_log, u_log.flatten(), label='LQR Control Effort')
plt.legend()
plt.grid()
plt.xlabel('Time (s)')

plt.tight_layout()
plt.show()
