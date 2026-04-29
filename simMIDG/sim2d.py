import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import random
import math
import heapq

def sim_run(options, _):
    MAP_W, MAP_H = options['MAP_SIZE']
    OBSTACLES = options['OBSTACLES']
    START = options['START']
    GOAL = options['GOAL']
    FIG_SIZE = options['FIG_SIZE']

    ROBOT_RADIUS = 0.05
    MIN_DIST_NODES = 0.06 

    def collision(x, y):
        if x < ROBOT_RADIUS or x > MAP_W - ROBOT_RADIUS: return True
        if y < ROBOT_RADIUS or y > MAP_H - ROBOT_RADIUS: return True
        for rect in OBSTACLES:
            rx, ry, rw, rh = rect
            closest_x = max(rx, min(x, rx + rw))
            closest_y = max(ry, min(y, ry + rh))
            dist = math.sqrt((x - closest_x)**2 + (y - closest_y)**2)
            if dist < ROBOT_RADIUS: return True
        return False

    # ==========================================
    # GENERAR NODOS
    # ==========================================
    nodes = [(START[0], START[1]), (GOAL[0], GOAL[1])]
    intentos = 0
    while len(nodes) < 1000 and intentos < 6000:
        intentos += 1
        x, y = random.uniform(0, MAP_W), random.uniform(0, MAP_H)
        if not collision(x, y):
            if all(math.sqrt((x-n[0])**2 + (y-n[1])**2) > MIN_DIST_NODES for n in nodes):
                nodes.append((x, y))

    start_node = nodes[0]
    goal_node = nodes[1]

    # ==========================================
    # GRAFO ALEATORIO (Para forzar rutas distintas)
    # ==========================================
    graph = {n: [] for n in nodes}
    def free_line(a, b, steps=25):
        for i in range(steps + 1):
            t = i / steps
            if collision(a[0] + t*(b[0]-a[0]), a[1] + t*(b[1]-a[1])): return False
        return True

    for node in nodes:
        candidates = [o for o in nodes if node != o and math.dist(node, o) < 0.5]
        vecinos = random.sample(candidates, min(len(candidates), 8))
        for near in vecinos:
            if free_line(node, near):
                graph[node].append(near)

    # ==========================================
    # A* ESTÁNDAR
    # ==========================================
    pq = [(0, start_node)]
    came_from = {}
    cost_so_far = {start_node: 0}
    while pq:
        _, current = heapq.heappop(pq)
        if current == goal_node: break
        for nxt in graph[current]:
            new_cost = cost_so_far[current] + math.dist(current, nxt)
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + math.dist(nxt, goal_node)
                heapq.heappush(pq, (priority, nxt))
                came_from[nxt] = current

    if goal_node not in came_from:
        return sim_run(options, _) 
    
    path = []
    curr = goal_node
    while curr != start_node:
        path.append(curr); curr = came_from[curr]
    path.append(start_node); path.reverse()

    # ==========================================
    # --- RESTAURADO: COMANDOS LEGO ---
    # ==========================================
    commands = []
    current_heading = START[2]
    trajectory_data = [] # Para la animación (x, y, ángulo)

    for i in range(1, len(path)):
        x1, y1 = path[i-1]; x2, y2 = path[i]
        target_heading = math.atan2(y2 - y1, x2 - x1)
        diff = (target_heading - current_heading + math.pi) % (2*math.pi) - math.pi
        
        if abs(diff) > 0.3:
            label = "GIRO_IZQUIERDA" if diff > 0 else "GIRO_DERECHA"
            commands.append((label, round(math.degrees(abs(diff)), 1)))
        
        dist_step = math.dist((x1, y1), (x2, y2))
        commands.append(("RECTO", round(dist_step, 2)))
        
        trajectory_data.append((x1, y1, target_heading))
        current_heading = target_heading
    
    trajectory_data.append((path[-1][0], path[-1][1], current_heading))

    print("\n--- COMANDOS LEGO PARA ESTA RUTA ---")
    for c in commands: print(c)

    # ==========================================
    # VISUALIZACIÓN COMPLETA
    # ==========================================
    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.set_xlim(0, MAP_W); ax.set_ylim(0, MAP_H); ax.set_aspect("equal")

    for obs in OBSTACLES:
        ax.add_patch(patches.Rectangle((obs[0], obs[1]), obs[2], obs[3], facecolor="black"))

    # Nodos visibles
    for node in nodes:
        ax.plot(node[0], node[1], "b.", markersize=3, alpha=0.4, zorder=2)

    # Rastro, Carrito y Línea de dirección
    trail, = ax.plot([], [], "r-", linewidth=1.5, alpha=0.5, zorder=3)
    history_x, history_y = [], []
    
    # --- RESTAURADO: Línea de dirección (la vueltita del círculo) ---
    dir_line, = ax.plot([], [], "r-", linewidth=2, zorder=6)
    car, = ax.plot([], [], "ro", markersize=10, zorder=5)

    def update(frame):
        x, y, th = trajectory_data[frame]
        history_x.append(x); history_y.append(y)
        
        trail.set_data(history_x, history_y)
        car.set_data([x], [y])
        
        # Actualizar flechita de dirección
        dx, dy = 0.1 * math.cos(th), 0.1 * math.sin(th)
        dir_line.set_data([x, x + dx], [y, y + dy])
        
        return car, trail, dir_line

    ani = animation.FuncAnimation(fig, update, frames=len(trajectory_data), interval=150, repeat=False)
    plt.title("PRM: Ruta Aleatoria + Comandos LEGO")
    plt.show()