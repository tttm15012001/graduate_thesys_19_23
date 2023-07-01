import numpy as np
from scipy.optimize import linear_sum_assignment

def calculate_distance(drone_coords, destinations):
    distances = []
    for drone in drone_coords:
        distances.append([np.linalg.norm(np.array(drone) - np.array(dest)) for dest in destinations])
    return distances

def minimize_path_cost(drone_coords, destinations):
    distances = calculate_distance(drone_coords, destinations)
    cost_matrix = np.array(distances)
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    assignments = [(drone_coords[row], destinations[col]) for row, col in zip(row_ind, col_ind)]
    total_cost = cost_matrix[row_ind, col_ind].sum()
    return assignments, total_cost

# Example usage
drone_coords = [[0, 0], [-200, 0], [-400, 0], [0, 200], [-200, 200], [-400, 200], [0, 400], [-200, 400], [-400, 400], [0, 600], [-200, 600], [-400, 600]]
destinations = [[1333.37, 610.72], [612.03, 1084.62], [972.69, 1558.54], [251.35, 2032.46], [697.12, 2258.93], [1142.89, 2485.43], [1588.65, 2711.91], [1418.46, 1311.12], [1503.56, 2011.50]]

assignments, total_cost = minimize_path_cost(drone_coords, destinations)
for assignment in assignments:
    drone, destination = assignment
    print("Drone:", drone, "assigned to Destination:", destination)
print("Total path cost:", total_cost)