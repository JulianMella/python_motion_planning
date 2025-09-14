import timeit
import pandas as pd
import sys, os
sys.path.insert(0, '/Users/anund/in5060/python_motion_planning/src')
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *
from datetime import datetime, time
import math
import random as rand


# 

algorithm_name = []
x_size = []
y_size = []
z_size = []
cost_array = []
search_area_array = []
length_array = []

def plt_in_array(plt, length):
    #global algorithm_name, x_size, y_size, z_size, cost_array, search_area_array
    cost, path, expand = plt.run()
    algorithm_name.append(plt.str())
    x_size.append(x)
    y_size.append(y)
    z_size.append(z)
    cost_array.append(cost)
    search_area_array.append(len(expand))
    length_array = length


if __name__ == '__main__':
    # Create environment with custom obstacles
    x = 40
    y = 40
    z = 5
    grid_env = Grid(x, y, z)
    
    basic_grid_obstacles = []
    base_pattern = [
        # Z = 1 (base layer - extended with holes)
        (2, 3, 1), (3, 3, 1), (4, 3, 1), (5, 3, 1), (6, 3, 1), (7, 3, 1), (8, 3, 1),
        (2, 4, 1), (3, 4, 1), (4, 4, 1), (6, 4, 1), (7, 4, 1), (8, 4, 1),  # hole at (5,4,1)
        (2, 5, 1), (3, 5, 1), (4, 5, 1), (5, 5, 1), (6, 5, 1), (7, 5, 1), (8, 5, 1),
        (2, 6, 1), (3, 6, 1), (5, 6, 1), (6, 6, 1), (7, 6, 1), (8, 6, 1),  # hole at (4,6,1)
        (2, 7, 1), (3, 7, 1), (4, 7, 1), (5, 7, 1), (6, 7, 1), (7, 7, 1), (8, 7, 1),
        # Z = 2 (middle layer - smaller)
        (4, 4, 2), (5, 4, 2), (6, 4, 2),
        (4, 5, 2), (5, 5, 2), (6, 5, 2),
        (4, 6, 2), (5, 6, 2), (6, 6, 2),
        # Z = 3 (top layer - pointy)
        (5, 5, 3)
    ]
    for i in range(4):  # 5 rows
        for j in range(4):  # 5 columns
            for x, y, z in base_pattern:
                new_x = x + (i * 10)
                new_y = y + (j * 10)
                if new_x < 40 and new_y < 40:  # Ensure we stay within grid bounds
                    basic_grid_obstacles.append((new_x, new_y, z))

    basic_grid_obstacles = list(set(basic_grid_obstacles))  # Remove any duplicates
    grid_env.inner_obstacles = basic_grid_obstacles


    common_start = (38,19,1)
    goals_short = []
    goals_long = []
    goals_long = []


    # Create 30 random start points
    #random_start = (rand.sample(range(1, 40), 30), rand.sample(range(1, 40), 30), rand.sample(range(1, 40), 30))
    random_start = [(rand.randint(1, 40), rand.randint(1, 40), rand.randint(1, 40)) for _ in range(30) if _ not in basic_grid_obstacles]



    #for start in random_start:
    start_x, start_y, start_z = common_start
    #start_x, start_y, start_z = start

    found_enough = False
    for x_coord in range(1, 40):
        if found_enough:
            break
        for y_coord in range(1, 40):
            if found_enough:
                break
            for z_coord in range(1, 5):
                # Calculate Euclidean distance
                distance = math.sqrt((x_coord - start_x)**2 + (y_coord - start_y)**2 + (z_coord - start_z)**2)
                # Check if distance is approximately 30 or 15 (with small tolerance for floating point)
                if abs(distance - 30.0) < 0.1 and (x_coord, y_coord, z_coord) not in basic_grid_obstacles and (x_coord, y_coord, z_coord) not in goals_long:
                    if len(goals_long) < 30:
                        goals_long.append((x_coord, y_coord, z_coord))
                elif abs(distance - 15.0) < 0.1 and (x_coord, y_coord, z_coord) not in basic_grid_obstacles and (x_coord, y_coord, z_coord) not in goals_short:
                    if len(goals_short) < 30:
                        goals_short.append((x_coord, y_coord, z_coord))
                
                if len(goals_long) >= 30 and len(goals_short) >= 30:
                    found_enough = True
                    break
                    
                    

    print(f"Made long goals. Amount: {len(goals_long)}")
    print(f"Made short goals. Amount: {len(goals_short)}")


   
    
    # algorithms we want to run
    algorithms = [
        ('Dijkstra', lambda start, goal, env: Dijkstra(start=start, goal=goal, env=env)),
        ('A*', lambda start, goal, env: AStar(start=start, goal=goal, env=env)),
        ('Theta*', lambda start, goal, env: ThetaStar(start=start, goal=goal, env=env)),
        ('JPS', lambda start, goal, env: JPS(start=start, goal=goal, env=env))
    ]
    x = 0

    time = []
    for goal in goals_long:

        for name, alg_func in algorithms:
            plt = alg_func(start=common_start, goal=goal, env=grid_env)
            start_time = timeit.default_timer()
            cost, expand = plt.run()
            end_time = timeit.default_timer()
            algorithm_name.append(plt.__str__())
            x_size.append(x)
            y_size.append(y)
            z_size.append(z)
            cost_array.append(cost)
            search_area_array.append(len(expand))
            length_array.append("long")
            x+=1
            time.append(end_time-start_time)
            print("Algo", x, "of", len(goals_long)*len(algorithms))



    for goal in goals_short:

        for name, alg_func in algorithms:
            plt = alg_func(start=common_start, goal=goal, env=grid_env)
            cost, expand = plt.run()
            algorithm_name.append(plt.__str__())
            x_size.append(x)
            y_size.append(y)
            z_size.append(z)
            cost_array.append(cost)
            search_area_array.append(len(expand))
            length_array.append("short")
            x+=1
            time.append(end_time-start_time)
            print("Algo", x, "of", len(goals_short)*len(algorithms))



    dict = {'algorithm': algorithm_name, 'x_size': x_size, 'y_size': y_size, 'z_size': z_size, 'cost': cost_array, 'search_area': search_area_array, 'radius': length_array, 'time':time}

    data_frame = pd.DataFrame(dict)
    print("reach")
    data_frame.to_csv('file1.csv', index=False)

    #plt.run()



# go through the algorithms automatically


# alle tester på samme grid
# på hver avstand må vi ha 20 forskjellige slutt, samme avstand, samme start 
# 

# list med starting points
# hvor hvert starting point, kalle på alle algoritmene
# 50 x 50 grid