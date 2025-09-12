import pandas as pd
import sys, os
sys.path.insert(0, '/Users/anund/in5060/python_motion_planning/src')
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *


# 

algorithm_name = []
x_size = []
y_size = []
z_size = []
cost_array = []
search_area_array = []

def plt_in_array(plt):
    #global algorithm_name, x_size, y_size, z_size, cost_array, search_area_array
    cost, path, expand = plt.run()
    algorithm_name.append(plt.str())
    x_size.append(x)
    y_size.append(y)
    z_size.append(z)
    cost_array.append(cost)
    search_area_array.append(len(expand))


if __name__ == '__main__':
    # Create environment with custom obstacles
    x = 20
    y = 20
    z = 10
    grid_env = Grid(x, y, z)

    basic_grid_obstacles = []
    base_pattern = [
        (4, 3, 1), (5, 3, 1), (6, 3, 1), (7, 3, 1),
        (4, 4, 1), (5, 4, 1), (3, 4, 1), (7, 4, 1),
        (4, 5, 1), (5, 5, 1), (3, 5, 1), (7, 5, 1),
        (4, 6, 1), (5, 6, 1), (3, 6, 1), (7, 6, 1),
        (4, 7, 1), (5, 7, 1), (3, 7, 1)
    ]
    for i in range(2):  # 5 rows
        for j in range(2):  # 5 columns
            for x, y, z in base_pattern:
                new_x = x + (i * 10)
                new_y = y + (j * 10)
                if new_x < 20 and new_y < 20:  # Ensure we stay within grid bounds
                    basic_grid_obstacles.append((new_x, new_y, z))

    basic_grid_obstacles = list(set(basic_grid_obstacles))  # Remove any duplicates
    grid_env.inner_obstacles = basic_grid_obstacles

    # inner_obstacles = set()
    # for i in range(x):
    #     for j in range(y):
    #         inner_obstacles.add((i, j, 7))
    #         inner_obstacles.add((i, j, 5))
    #         inner_obstacles.add((i, j, 3))

    # inner_obstacles.remove((8,8,3))
    # inner_obstacles.remove((1,1,5))
    # inner_obstacles.remove((5,5,7))

    # grid_env.inner_obstacles = inner_obstacles

    # grid_env.inner_obstacles.clear()

    common_start = (18,10,1)
    goals_short = [(1,17,1),(1,17,8),(1,16,8),(1,15,8),(1,14,8),(1,13,8),(1,12,8)]
    goals_long = []

    x = 8
    y = 8
    z = 8

    goals = []

    for idx in range (x):
        for idy in range(y):
            for idz in range(z):
                point = (idx,idy,idz)
                goals.append(point)
    print("length of list goals is: ", len(goals))
   
    
    # algorithms we want to run
    algorithms = [
        #('Dijkstra', lambda start, goal, env: Dijkstra(start=start, goal=goal, env=env)),
        #('A*', lambda start, goal, env: AStar(start=start, goal=goal, env=env)),
        ('Theta*', lambda start, goal, env: ThetaStar(start=start, goal=goal, env=env))
    ]
    x = 0
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
            x+=1
            print(x,"of",len(goals) )
            break


      
    #   print(cost)

    dict = {'algorithm': algorithm_name, 'x_size': x_size, 'y_size': y_size, 'z_size': z_size, 'cost': cost_array, 'search_area': search_area_array}
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