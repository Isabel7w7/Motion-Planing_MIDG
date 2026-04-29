from simMIDG.sim2d import sim_run

# ==========================================
# OPCIONES DEL SIMULADOR
# ==========================================
options = {}

options['FIG_SIZE'] = [8, 8]
options['MAP_SIZE'] = [2.0, 2.0]

# Obstáculos (x, y, ancho, alto)
options['OBSTACLES'] = [
    [0.0, 1.5, 1.0, 0.5],   # superior izquierdo
    [0.4, 0.5, 0.6, 0.5],   # centro
    [1.5, 0.5, 0.2, 1.0],   # vertical derecho
    [1.0, 0.0, 1.0, 0.15]   # inferior
]

# Inicio y meta
options['START'] = (0.15, 0.06, 0.0)
options['GOAL'] = (1.92, 1.75)

# Ejecutar simulación
sim_run(options, None)