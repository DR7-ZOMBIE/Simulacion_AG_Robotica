import numpy as np
import random
import matplotlib.pyplot as plt
from deap import base, creator, tools
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import math

# Parámetros del problema
NUM_POINTS = int(math.factorial(5) / 2)  # Número de puntos de tarea
DOFs = 2  # Grados de libertad del manipulador
IKM_CONFIGS = 2 ** NUM_POINTS  # Configuraciones posibles de cinemática inversa
PLACEMENT_NODES = 6  # Posibles ubicaciones del manipulador
ORIENTATION_NODES = 1000  # Posibles orientaciones del manipulador
LINK_LENGTHS = [1, 1]  # Longitudes de los enlaces del brazo robótico

def evaluate(individual):
    # Extraer los segmentos del cromosoma
    task_order = individual[:NUM_POINTS]
    ikm_configs = individual[NUM_POINTS:NUM_POINTS * 2]
    placements = individual[NUM_POINTS * 2:NUM_POINTS * 3]
    orientations = individual[NUM_POINTS * 3:]
    
    # Calcular el tiempo de ciclo basado en los segmentos del cromosoma
    cycle_time = 0
    for i in range(len(task_order) - 1):
        # Asegurar que los valores sean enteros antes de la resta
        displacement_time = np.abs(task_order[i+1] - task_order[i]) / (1 + ikm_configs[i])
        placement_factor = placements[i] / PLACEMENT_NODES
        orientation_factor = orientations[i] / ORIENTATION_NODES
        cycle_time += displacement_time * (1 + placement_factor + orientation_factor)
    
    return cycle_time,

# Configurar el tipo de cromosoma y la función de evaluación
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()

# Atributos de los individuos: tareas, configuraciones, ubicaciones, orientaciones
toolbox.register("attr_task", random.sample, range(NUM_POINTS), NUM_POINTS)
toolbox.register("attr_ikm", random.randint, 0, IKM_CONFIGS-1)
toolbox.register("attr_place", random.randint, 0, PLACEMENT_NODES-1)
toolbox.register("attr_orient", random.randint, 0, ORIENTATION_NODES-1)

# Definición del individuo y la población
def init_individual(icls):
    task_order = toolbox.attr_task()
    ikm_configs = [toolbox.attr_ikm() for _ in range(NUM_POINTS)]
    placements = [toolbox.attr_place() for _ in range(NUM_POINTS)]
    orientations = [toolbox.attr_orient() for _ in range(NUM_POINTS)]
    return icls(task_order + ikm_configs + placements + orientations)

toolbox.register("individual", init_individual, creator.Individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Registro de operadores de algoritmo genético
toolbox.register("mate", tools.cxUniform, indpb=0.5)
toolbox.register("mutate", tools.mutUniformInt, low=0, up=max(IKM_CONFIGS-1, PLACEMENT_NODES-1, ORIENTATION_NODES-1), indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)
toolbox.register("evaluate", evaluate)

def main():
    # Crear una población inicial
    population = toolbox.population(n=50)
    
    # Definir los parámetros del algoritmo genético
    NGEN = 800  # Número de generaciones
    CXPB = 0.7  # Probabilidad de cruce
    MUTPB = 0.2  # Probabilidad de mutación
    
    # Listas para almacenar valores durante la simulación
    best_fitness_values = []
    avg_fitness_values = []
    diversity_values = []
    
    # Algoritmo genético: evaluación inicial
    fitnesses = list(map(toolbox.evaluate, population))
    for ind, fit in zip(population, fitnesses):
        ind.fitness.values = fit
    
    # Ciclo del algoritmo genético
    for gen in range(NGEN):
        offspring = toolbox.select(population, len(population))
        offspring = list(map(toolbox.clone, offspring))
        
        # Aplicar cruce y mutación
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values
        
        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values
        
        # Evaluar individuos con fitness no calculado
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
        
        # Reemplazar la población por la nueva generación
        population[:] = offspring
        
        # Almacenar valores de fitness
        fits = [ind.fitness.values[0] for ind in population]
        best_fitness_values.append(min(fits))
        avg_fitness_values.append(sum(fits) / len(population))
        diversity_values.append(len(set(fits)))
    
    # Obtener y mostrar el mejor resultado
    best_ind = tools.selBest(population, 1)[0]
    print(f"Mejor individuo: {best_ind}")
    print(f"Tiempo mínimo de ciclo: {best_ind.fitness.values[0]}")
    
    # Graficar la evolución del mejor tiempo de ciclo
    plt.figure(figsize=(10, 6))
    plt.plot(best_fitness_values, label='Mejor tiempo de ciclo')
    plt.xlabel('Generación')
    plt.ylabel('Tiempo de ciclo')
    plt.title('Evolución del mejor tiempo de ciclo a través de las generaciones')
    plt.legend()
    plt.grid()
    plt.show()

    # Graficar la evolución del tiempo promedio de ciclo
    plt.figure(figsize=(10, 6))
    plt.plot(avg_fitness_values, label='Tiempo promedio de ciclo', color='orange')
    plt.xlabel('Generación')
    plt.ylabel('Tiempo promedio de ciclo')
    plt.title('Evolución del tiempo promedio de ciclo a través de las generaciones')
    plt.legend()
    plt.grid()
    plt.show()

    # Graficar la diversidad de la población
    plt.figure(figsize=(10, 6))
    plt.plot(diversity_values, label='Diversidad de la población', color='green')
    plt.xlabel('Generación')
    plt.ylabel('Número de soluciones únicas')
    plt.title('Evolución de la diversidad de la población')
    plt.legend()
    plt.grid()
    plt.show()

    # Graficar la distribución de tiempos de ciclo en la última generación
    plt.figure(figsize=(10, 6))
    plt.hist(fits, bins=20, color='purple', alpha=0.7)
    plt.xlabel('Tiempo de ciclo')
    plt.ylabel('Frecuencia')
    plt.title('Distribución de tiempos de ciclo en la última generación')
    plt.grid()
    plt.show()

def forward_kinematics(joint_angles, link_lengths):
    """
    Calcular la posición final del end-effector del brazo robótico usando cinemática directa.
    Utiliza un modelo de brazo robótico con tres grados de libertad.
    """
    x = [0]
    y = [0]
    z = [0]
    
    for i in range(len(joint_angles)):
        theta = np.radians(joint_angles[i])
        x.append(x[-1] + link_lengths[i] * np.cos(theta))
        y.append(y[-1] + link_lengths[i] * np.sin(theta))
        z.append(0)  # Mantener z en cero para simplificación; se puede ajustar para brazos 3D
    
    return np.array(x), np.array(y), np.array(z)

def plot_robot_arm(joint_angles, link_lengths):
    """Función para graficar el brazo robótico dado un conjunto de ángulos de las articulaciones."""
    x, y, z = forward_kinematics(joint_angles, link_lengths)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Graficar los enlaces del brazo robótico
    ax.plot(x, y, z, '-o', markersize=10, label='Robot Arm', color='blue')
    
    # Mejorar la visualización de las articulaciones
    for i in range(len(x)):
        ax.scatter(x[i], y[i], z[i], color='red', s=100)
    
    ax.set_xlim(-sum(link_lengths), sum(link_lengths))
    ax.set_ylim(-sum(link_lengths), sum(link_lengths))
    ax.set_zlim(-sum(link_lengths), sum(link_lengths))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Simulación del Brazo Robótico Mejorado')
    ax.legend()
    plt.show()

def animate_robot_arm(joint_angles_list, link_lengths):
    """Función para animar el movimiento del brazo robótico con una animación más suave."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    x, y, z = forward_kinematics(joint_angles_list[0], link_lengths)
    line, = ax.plot(x, y, z, '-o', markersize=10)

    def update(frame):
        x, y, z = forward_kinematics(joint_angles_list[frame], link_lengths)
        line.set_data(x, y)
        line.set_3d_properties(z)
        return line,

    ani = FuncAnimation(fig, update, frames=len(joint_angles_list), blit=True, interval=200)
    ax.set_xlim(-sum(link_lengths), sum(link_lengths))
    ax.set_ylim(-sum(link_lengths), sum(link_lengths))
    ax.set_zlim(-sum(link_lengths), sum(link_lengths))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Simulación del Brazo Robótico en Movimiento')
    plt.show()

def simulate_robot_arm_movement(best_individual, link_lengths):
    """Simula el movimiento del brazo robótico basado en la trayectoria óptima encontrada."""
    # Extract the joint angles from the best individual
    joint_angles_list = []
    for ikm_config in best_individual[NUM_POINTS:NUM_POINTS * 2]:
        joint_angles = np.random.uniform(low=-180, high=180, size=DOFs)  # Simulación de ángulos de articulación
        joint_angles_list.append(joint_angles)
    
    # Animar el movimiento del brazo robótico
    animate_robot_arm(joint_angles_list, link_lengths)

if __name__ == "__main__":
    main()

    # Simulación del brazo robótico
    simulate_robot_arm_movement(tools.selBest(toolbox.population(n=50), 1)[0], LINK_LENGTHS)
