import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from scipy.spatial import KDTree
import math
import random

# ==========================================
# CONFIGURACIÓN (Sin cambios)
# ==========================================
MAP_W, MAP_H = 2.0, 2.0
OBSTACLES = [
    [0.0, 1.5, 1.0, 0.5], [0.4, 0.5, 0.6, 0.5],
    [1.5, 0.5, 0.2, 1.0], [1.0, 0.0, 1.0, 0.15]
]
WHEEL_BASE = 0.18
WHEEL_RADIUS = 0.03
ROBOT_RADIUS = 0.05
START = (0.15, 0.06, 0.0)
GOAL = (1.92, 1.75)

N_SAMPLES = 1000
MIN_DIST = 0.07  
MAX_CONN_DIST = 0.45 

# ==========================================
# FUNCIONES BÁSICAS (Sin cambios)
# ==========================================
def collision(x, y):
    if x < ROBOT_RADIUS or x > MAP_W - ROBOT_RADIUS: return True
    if y < ROBOT_RADIUS or y > MAP_H - ROBOT_RADIUS: return True
    for rect in OBSTACLES:
        rx, ry, rw, rh = rect
        cx = max(rx, min(x, rx + rw))
        cy = max(ry, min(y, ry + rh))
        if math.sqrt((x - cx)**2 + (y - cy)**2) < ROBOT_RADIUS: return True
    return False

# ==========================================
# GENERACIÓN DE PRM UNIFORME (Sin cambios)
# ==========================================
def generate_prm():
    samples = [START[:2], GOAL]
    attempts = 0
    while len(samples) < N_SAMPLES and attempts < 5000:
        attempts += 1
        px, py = np.random.uniform(0, MAP_W), np.random.uniform(0, MAP_H)
        if not collision(px, py):
            dist, _ = KDTree(samples).query([px, py])
            if dist > MIN_DIST:
                samples.append((px, py))
    
    samples = np.array(samples)
    tree = KDTree(samples)
    adj = {i: [] for i in range(len(samples))}
    
    for i, s in enumerate(samples):
        indices = tree.query_ball_point(s, MAX_CONN_DIST)
        for idx in indices:
            if i != idx:
                mid_x, mid_y = (s[0]+samples[idx][0])/2, (s[1]+samples[idx][1])/2
                if not collision(mid_x, mid_y):
                    d = math.dist(s, samples[idx])
                    adj[i].append((idx, d))
    return samples, adj

# ==========================================
# BÚSQUEDA DE RUTA ALEATORIA (Sin cambios)
# ==========================================
def find_random_path(samples, adj):
    start_idx, goal_idx = 0, 1
    queue = [(start_idx, [])]
    visited = set()
    
    while queue:
        random.shuffle(queue)
        u, path = queue.pop(0)
        if u in visited: continue
        visited.add(u)
        if u == goal_idx: return path + [u]
        neighbors = adj[u][:]
        random.shuffle(neighbors)
        for v, weight in neighbors:
            if v not in visited:
                queue.append((v, path + [u]))
    return None

# ==========================================
# EJECUCIÓN PRINCIPAL (Sin cambios)
# ==========================================
samples, adj = generate_prm()
path_indices = find_random_path(samples, adj)

if not path_indices:
    print("No se encontró ruta. ¡Intenta de nuevo!")
    exit()

trajectory = []
print("\n--- RUTA ALEATORIA GENERADA ---")
for i in range(len(path_indices)-1):
    p1 = samples[path_indices[i]]
    p2 = samples[path_indices[i+1]]
    angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    trajectory.append((p1[0], p1[1], angle))
    dist = math.dist(p1, p2)
    print(f"Paso {i+1}: Movimiento hacia ({p2[0]:.2f}, {p2[1]:.2f}), Dist: {dist:.2f}m")

trajectory.append((GOAL[0], GOAL[1], trajectory[-1][2]))

# ==========================================
# ANIMACIÓN (Con línea de seguimiento)
# ==========================================
fig, ax = plt.subplots(figsize=(7,7))
ax.set_xlim(0, MAP_W); ax.set_ylim(0, MAP_H); ax.set_aspect("equal")

for obs in OBSTACLES:
    ax.add_patch(patches.Rectangle((obs[0], obs[1]), obs[2], obs[3], facecolor="black"))

ax.scatter(samples[:,0], samples[:,1], s=5, color="blue", alpha=0.1) # Bajada opacidad puntos
ax.plot(START[0], START[1], "ro", markersize=10, zorder=5)
ax.plot(GOAL[0], GOAL[1], "go", markersize=10, zorder=5)

# --- NUEVO: Inicializar objeto de rastro (línea continua roja suave) ---
trail_line, = ax.plot([], [], "r-", linewidth=1, alpha=0.3, zorder=3)
# Historial de posiciones
history_x = []
history_y = []

# Línea de dirección del robot (puntero)
robot_dir_line, = ax.plot([], [], "r-", linewidth=2, zorder=4)
# Cuerpo del robot
robot_body = patches.Circle((0,0), ROBOT_RADIUS, color="red", alpha=0.6, zorder=4)
ax.add_patch(robot_body)

def update(frame):
    x, y, th = trajectory[frame]
    
    # --- NUEVO: Guardar posición actual en el historial ---
    history_x.append(x)
    history_y.append(y)
    
    # --- NUEVO: Actualizar los datos del rastro con todo el historial ---
    trail_line.set_data(history_x, history_y)
    
    # Actualizar cuerpo del robot
    robot_body.center = (x, y)
    
    # Actualizar puntero de dirección
    dx, dy = 0.1 * math.cos(th), 0.1 * math.sin(th)
    robot_dir_line.set_data([x, x+dx], [y, y+dy])
    
    # --- MODIFICADO: Retornar también trail_line ---
    return robot_body, robot_dir_line, trail_line

# Reducido el intervalo a 100 para que el rastro se dibuje más rápido
ani = animation.FuncAnimation(fig, update, frames=len(trajectory), interval=100, repeat=False, blit=True)
plt.title("Exploración Aleatoria sobre PRM con Línea de Seguimiento")
plt.show()