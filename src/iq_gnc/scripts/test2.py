import random
import math

# PSO parameters
num_particles = 50  # Number of particles in the swarm
max_iterations = 100  # Maximum number of iterations
c1 = 2.0  # Cognitive parameter
c2 = 2.0  # Social parameter
w = 0.7  # Inertia weight

# Problem-specific parameters
num_drones = 5  # Number of drones
drone_coords = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10)]
dest_coords = [(200, 150), (180, 460), (230, 190), (27, 18), (85, 9)]

def update_particle_velocity(particle, swarm_best_position):
    # Update the velocity of a particle
    for i in range(num_drones):
        r1 = random.random()
        r2 = random.random()
        cognitive_component = c1 * r1 * (particle.best_position[i] - particle.position[i])
        social_component = c2 * r2 * (swarm_best_position[i] - particle.position[i])
        particle.velocity[i] = w * particle.velocity[i] + cognitive_component + social_component


def update_particle_position(particle):
    # Update the position of a particle
    for i in range(num_drones):
        particle.position[i] += int(round(particle.velocity[i]))
        # Clamp the position within the bounds (0 to num_drones - 1)
        particle.position[i] = max(0, min(particle.position[i], num_drones - 1))


def update_swarm_best(swarm, swarm_best_position):
    # Update the swarm's best position
    for particle in swarm:
        if particle.best_fitness < evaluate_fitness(swarm_best_position):
            swarm_best_position = list(particle.best_position)
    return swarm_best_position


class Particle:
    def __init__(self):
        self.position = []  # Current position (drone assignments)
        self.best_position = []  # Personal best position
        self.velocity = [] # Velocity
        self.best_fitness = float('inf')  # Personal best fitness


def evaluate_fitness(position):
    total_distance = 0
    for i in range(num_drones):
        drone_x, drone_y = drone_coords[i]
        dest_x, dest_y = dest_coords[position[i]]
        distance = math.sqrt((dest_x - drone_x) ** 2 + (dest_y - drone_y) ** 2)
        total_distance += distance
    return total_distance


def pso():
    # Initialize the swarm
    swarm = []
    swarm_best_position = []
    swarm_best_fitness = float('inf')

    for _ in range(num_particles):
        particle = Particle()
        particle.position = random.sample(range(len(dest_coords)), num_drones)  # Random initial assignments

        # Update personal best position and fitness
        particle.best_position = list(particle.position)
        particle.best_fitness = evaluate_fitness(particle.position)

        # Update swarm's best position and fitness
        if particle.best_fitness < swarm_best_fitness:
            swarm_best_position = list(particle.position)
            swarm_best_fitness = particle.best_fitness

        swarm.append(particle)

    # Run PSO iterations
    for _ in range(max_iterations):
        for particle in swarm:
            # update_particle_position(particle)

            # Ensure no duplicate assignments
            # while len(set(particle.position)) != num_drones:
            #     update_particle_position(particle)

            # Update personal best position and fitness
            fitness = evaluate_fitness(particle.position)
            if fitness < particle.best_fitness:
                particle.best_position = list(particle.position)
                particle.best_fitness = fitness

        # Update swarm's best position and fitness
        swarm_best_position = update_swarm_best(swarm, swarm_best_position)

    # Extract the best solution
    best_position = swarm_best_position

    # Print the best solution
    print("Best solution found:")
    for i, drone in enumerate(drone_coords):
        drone_index = best_position[i]
        destination = dest_coords[drone_index]
        print(f"Drone {i+1}: Assigned to destination {destination}")

    return best_position


# Run PSO algorithm
best_solution = pso()