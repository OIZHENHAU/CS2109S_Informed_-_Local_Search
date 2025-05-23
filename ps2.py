""" CS2109S Problem Set 2: Informed Search"""

import copy
import heapq
import math
import random
import time
from typing import List, Tuple

import cube
import utils
import copy
import json
import cube
import utils
from ast import literal_eval
from utils import PriorityQueue
from utils import Node
from typing import Dict, Iterable, List, Optional, Tuple, Union
Action = List[Union[int, str]]

""" ADD HELPER FUNCTION HERE """

"""
We provide implementations for the Node and PriorityQueue classes in utils.py, but you can implement your own if you wish
"""
from utils import Node
from utils import PriorityQueue


#TODO Task 1.1: Implement your heuristic function, which takes in an instance of the Cube and
#   the State class and returns the estimated cost of reaching the goal state from this state.
def heuristic_func(problem: cube.Cube, state: cube.State) -> float:
    h_n = 0
    goals = problem.goal
    initial_layout = state.layout
    goal_layout = goals.layout
    matrix = goals.shape
    for i in range(matrix[0] * matrix[1]):
        if (initial_layout[i] != goal_layout[i]):
            h_n += 1
    h_n = h_n / 2
    return h_n

# Test
def wrap_test(func):
    def inner(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            return f'FAILED, error: {type(e).__name__}, reason: {str(e)}'
    return inner

# Test case for Task 1.1
@wrap_test
def test_heuristic(case):

    input_dict = case['input_dict']
    answer = case['answer']
    problem = cube.Cube(input_dict = input_dict)

    assert heuristic_func(problem, problem.goal) == 0, "Heuristic is not 0 at the goal state"
    assert heuristic_func(problem, problem.initial) <= answer['cost'], "Heuristic is not admissible"

    return "PASSED"

if __name__ == '__main__':
    
    cube1 = {'input_dict': {"initial": {'shape': [3, 3], 'layout': ['N', 'U',   
        'S', 'N','U', 'S', 'N', 'U', 'S']}, 'goal': {'shape': [3, 3], 'layout': 
        ['N', 'U', 'S', 'N', 'U', 'S', 'N', 'U', 'S']}}, 'answer': {'solution': 
        [], 'cost': 0}}

    cube2 = {'input_dict': {"initial": {'shape': [3, 3], 'layout': ['S', 'O', 
        'C', 'S', 'O', 'C', 'S', 'O', 'C']}, 'goal': {'shape': [3, 3], 
        'layout': ['S', 'S', 'S', 'O', 'O', 'O', 'C', 'C', 'C']}}, 'answer': 
        {'solution': [[2, 'right'], [1, 'left'], [1, 'down'], 
        [2, 'up']], 'cost': 4}}

    cube3 = {'input_dict': {"initial": {'shape': [3, 3], 'layout': ['N', 'U',   
        'S', 'N', 'U', 'S', 'N', 'U', 'S']}, 'goal': {'shape': [3, 3], 'layout': 
        ['S', 'U', 'N', 'N', 'S', 'U', 'U', 'N', 'S']}}, 'answer': {'solution': 
        [[0, 'left'], [1, 'right'], [0, 'up'], [1, 'down']], 'cost': 4}}

    cube4 = {'input_dict':{"initial": {'shape': [3, 4], 'layout': [1, 1, 9, 0,
        2, 2, 0, 2, 9, 0, 1, 9]}, 'goal': {'shape': [3, 4], 'layout': [ 1, 0,
        9, 2, 2, 1, 0, 9, 2, 1, 0, 9]}}, 'answer': {'solution': [[1, 'down'],
        [3, 'up'], [2, 'left']], 'cost': 3}}

    print('Task 1.1:')
    print('cube1: ' + test_heuristic(cube1))
    print('cube2: ' + test_heuristic(cube2))
    print('cube3: ' + test_heuristic(cube3))
    print('cube4: ' + test_heuristic(cube4))
    print('\n')


#TODO Task 1.2: Implement A* search which takes in an instance of the Cube 
#   problem class and returns a list of actions [(2,'left'), ...].
def astar_search(problem: cube.Cube):
    r"""
    A* Search finds the solution to reach the goal from the initial.
    If no solution is found, return False.

    Args:
        problem (cube.Cube): Cube instance

    Returns:
        solution (List[Action]): the action sequence
    """
    fail = True
    solution = []
    queue = utils.PriorityQueue()
    initial_problem = problem.initial
    goals = problem.goal
    initial_state = cube.State(initial_problem.shape, initial_problem.layout)
    # print(initial_state)
    h_n = heuristic_func(problem, initial_problem)
    # print(h_n)
    g_n = 0
    initial_node = utils.Node(None, solution, initial_state, g_n, h_n)
    # print(initial_node)
    queue.push(g_n + h_n, initial_node)
    # print(queue)
    while queue:
        curr_node = queue.pop()
        # print(curr_node.state)
        if (problem.goal_test(curr_node.state)):
            fail = False
            solution = curr_node.act
            return solution
        else:
            new_actions = problem.actions(curr_node.act)
            # print(new_actions)
            for action in new_actions:
                # print(action)
                new_state = problem.result(curr_node.state, action)
                # print(new_state)
                new_g_n = problem.path_cost(curr_node.g_n, curr_node.state, action, new_state)
                # print(new_g_n)
                new_h_n = heuristic_func(problem, new_state)
                # print(new_h_n)
                new_act = curr_node.act + [action]
                new_node = utils.Node(curr_node, new_act, new_state, new_g_n, new_h_n)
                # print(new_node)
                new_f_n = new_g_n + new_h_n
                queue.push(new_f_n, new_node)
    if fail:
        return False
    return solution

@wrap_test
def test_astar(case):

    input_dict = case['input_dict']
    answer = case['answer']
    problem = cube.Cube(input_dict = input_dict)
    
    start = time.time()
    solution = astar_search(problem)
    print(f"Time lapsed: {time.time() - start}")

    if solution is False:
        assert answer['solution'] is False, "Solution is not False"
    else:
        correctness, cost = problem.verify_solution(solution, _print=False)
        assert correctness, f"Fail to reach goal state with solution {solution}"
        assert cost <= answer['cost'], f"Cost is not optimal."
    return "PASSED"


if __name__ == '__main__':
    
    cube1 = {'input_dict': {"initial": {'shape': [3, 3], 'layout': ['N', 'U',   
        'S', 'N','U', 'S', 'N', 'U', 'S']}, 'goal': {'shape': [3, 3], 'layout': 
        ['N', 'U', 'S', 'N', 'U', 'S', 'N', 'U', 'S']}}, 'answer': {'solution': 
        [], 'cost': 0}}

    cube2 = {'input_dict': {"initial": {'shape': [3, 3], 'layout': ['S', 'O', 
        'C', 'S', 'O', 'C', 'S', 'O', 'C']}, 'goal': {'shape': [3, 3], 
        'layout': ['S', 'S', 'S', 'O', 'O', 'O', 'C', 'C', 'C']}}, 'answer': 
        {'solution': [[2, 'right'], [1, 'left'], [1, 'down'], 
        [2, 'up']], 'cost': 4}}

    cube3 = {'input_dict': {"initial": {'shape': [3, 3], 'layout': ['N', 'U',   
        'S', 'N', 'U', 'S', 'N', 'U', 'S']}, 'goal': {'shape': [3, 3], 'layout': 
        ['S', 'U', 'N', 'N', 'S', 'U', 'U', 'N', 'S']}}, 'answer': {'solution': 
        [[0, 'left'], [1, 'right'], [0, 'up'], [1, 'down']], 'cost': 4}}

    cube4 = {'input_dict':{"initial": {'shape': [3, 4], 'layout': [1, 1, 9, 0,
        2, 2, 0, 2, 9, 0, 1, 9]}, 'goal': {'shape': [3, 4], 'layout': [ 1, 0,
        9, 2, 2, 1, 0, 9, 2, 1, 0, 9]}}, 'answer': {'solution': [[1, 'down'],
        [3, 'up'], [2, 'left']], 'cost': 3}}

    print('Task 1.2:')
    print('cube1: ' + test_astar(cube1)) 
    print('cube2: ' + test_astar(cube2)) 
    print('cube3: ' + test_astar(cube3)) 
    print('cube4: ' + test_astar(cube4)) 
    print('\n')


#TODO Task 1.3: Explain why the heuristic you designed for Task 1.1 is {consistent} 
#   and {admissible}.


#TODO Task 2.1: Propose a state representation for this problem if we want to formulate it
#   as a local search problem.


#TODO Task 2.2: What are the initial and goal states under your proposed representation?


#TODO Task 2.3: Define a reasonable transition function to generate new candidate solutions.
def transition(route: List[int]):
    route_result = []
    rr = route
    # print(rr)
    for i in range(len(route)):
        curr_route = rr.copy()
        i1, i2 = random.sample(range(len(route)), 2)
        const = curr_route[i1]
        curr_route[i1] = curr_route[i2]
        curr_route[i2] = const
        route_result.append(curr_route)
    # print(route_result)
    return route_result

# Test
@wrap_test
def test_transition(route: List[int]):
    for new_route in transition(route):
        assert sorted(new_route) == list(range(len(route))), "Invalid route"

    return "PASSED"

if __name__ == '__main__':
    print('Task 2.3:')
    print(test_transition([1, 3, 2, 0]))
    print(test_transition([7, 8, 6, 3, 5, 4, 9, 2, 0, 1]))
    print('\n')


#TODO Task 2.4: Implement a heuristic to decide on the "goodness" of a route.
from typing import List, Tuple
import random


def heuristic(cities: int, distances: List[Tuple[int]], route: List[int]) -> int:
    def adjMatrix(cities, distances):
        matrix = []
        for i in range(cities):
            # print(i)
            matrix.append([])
            for j in range(cities):
                matrix[i].append(0)
        for (a, b, c) in distances:
            # print(a, b, c)
            matrix[a][b] = c
            matrix[b][a] = c
        return matrix

    adj_matrix = adjMatrix(cities, distances)
    h_n = 0
    route.append(route[0])
    length_route = len(route)
    for i in range(length_route - 1):
        x = route[i]
        y = route[i + 1]
        h_n += (30 - adj_matrix[x][y])

    return h_n

if __name__ == '__main__':
    cities = 4
    distances = [(1, 0, 10), (0, 3, 22), (2, 1, 8), (2, 3, 30), (1, 3, 25), (0, 2, 15)]

    route_1 = heuristic(cities, distances, [0, 1, 2, 3])
    route_2 = heuristic(cities, distances, [2, 1, 3, 0])
    route_3 = heuristic(cities, distances, [1, 3, 2, 0])

    print('Task 2.4:')
    print(route_1 == route_2)  # True
    print(route_1 > route_3)  # True
    print('\n')


#TODO Task 2.5: Explain why your heuristic is suitable for this problem.


#TODO Task 2.6: Implement hill-climbing which takes in the number of cities and the list of
#   distances and returns the shortest route as a list of cities.
from typing import List, Tuple
import random


def transition(route: List[int]):
    route_result = []
    rr = route
    # print(rr)
    for i in range(len(route)):
        curr_route = rr.copy()
        i1, i2 = random.sample(range(len(route)), 2)
        const = curr_route[i1]
        curr_route[i1] = curr_route[i2]
        curr_route[i2] = const
        route_result.append(curr_route)
    # print(route_result)
    return route_result


def heuristic(cities: int, distances: List[Tuple[int]], route: List[int]) -> int:
    def adjMatrix(cities, distances):
        matrix = []
        for i in range(cities):
            # print(i)
            matrix.append([])
            for j in range(cities):
                matrix[i].append(0)
        for (a, b, c) in distances:
            # print(a, b, c)
            matrix[a][b] = c
            matrix[b][a] = c
        return matrix

    adj_matrix = adjMatrix(cities, distances)
    h_n = 0
    route.append(route[0])
    length_route = len(route)
    for i in range(length_route - 1):
        x = route[i]
        y = route[i + 1]
        h_n += (30 - adj_matrix[x][y])

    return h_n


def hill_climbing(cities: int, distances: List[Tuple[int]]):
    solution = []
    curr_route = []
    # print(route)
    route_cost = 999
    random_route = transition(list(range(cities)))
    # print(random_route)
    for i in range(cities - 1):
        curr_route_1 = random_route[i]
        # print(curr_route_1)
        curr_route_2 = random_route[i + 1]
        # print(curr_route_2)
        curr_cost_1 = heuristic(cities, distances, curr_route_1)
        # print(curr_cost_1)
        curr_cost_2 = heuristic(cities, distances, curr_route_2)
        # print(curr_cost_2)
        if curr_cost_1 <= curr_cost_2:
            if route_cost > curr_cost_1:
                curr_route = curr_route_1
                # print(curr_route)
                route_cost = curr_cost_1
                # print(route_cost)
        else:
            if route_cost > curr_cost_2:
                curr_route = curr_route_2
                # print(curr_route)
                route_cost = curr_cost_2
                # print(route_cost)
    # print(curr_route)
    for i in range(cities):
        solution.append(curr_route[i])
    # print(solution)
    return solution

# Test
@wrap_test
def test_hill_climbing(cities: int, distances: List[Tuple[int]]):
    start = time.time()
    route = hill_climbing(cities, distances)
    print(f"Time lapsed: {time.time() - start}")

    assert sorted(route) == list(range(cities)), "Invalid route"

    return "PASSED"

if __name__ == '__main__':
    
    cities_1 = 4
    distances_1 = [(1, 0, 10), (0, 3, 22), (2, 1, 8), (2, 3, 30), (1, 3, 25), (0, 2, 15)]

    cities_2 = 10
    distances_2 = [(2, 7, 60), (1, 6, 20), (5, 4, 70), (9, 8, 90), (3, 7, 54), (2, 5, 61),
        (4, 1, 106), (0, 6, 51), (3, 1, 45), (0, 5, 86), (9, 2, 73), (8, 4, 14), (0, 1, 51),
        (9, 7, 22), (3, 2, 22), (8, 1, 120), (5, 7, 92), (5, 6, 60), (6, 2, 10), (8, 3, 78),
        (9, 6, 82), (0, 2, 41), (2, 8, 99), (7, 8, 71), (0, 9, 32), (4, 0, 73), (0, 3, 42),
        (9, 1, 80), (4, 2, 85), (5, 9, 113), (3, 6, 28), (5, 8, 81), (3, 9, 72), (9, 4, 81),
        (5, 3, 45), (7, 4, 60), (6, 8, 106), (0, 8, 85), (4, 6, 92), (7, 6, 70), (7, 0, 22),
        (7, 1, 73), (4, 3, 64), (5, 1, 80), (2, 1, 22)]

    print('Task 2.6:')
    print('cities_1: ' + test_hill_climbing(cities_1, distances_1))
    print('cities_2: ' + test_hill_climbing(cities_2, distances_2))
    print('\n')


#TODO Task 2.7: Implement hill_climbing_with_random_restarts by repeating hill climbing
#   at different random locations.
from typing import List, Tuple
import random


def transition(route: List[int]):
    route_result = []
    rr = route
    # print(rr)
    for i in range(len(route)):
        curr_route = rr.copy()
        i1, i2 = random.sample(range(len(route)), 2)
        const = curr_route[i1]
        curr_route[i1] = curr_route[i2]
        curr_route[i2] = const
        route_result.append(curr_route)
    # print(route_result)
    return route_result


def heuristic(cities: int, distances: List[Tuple[int]], route: List[int]) -> int:
    def adjMatrix(cities, distances):
        matrix = []
        for i in range(cities):
            # print(i)
            matrix.append([])
            for j in range(cities):
                matrix[i].append(0)
        for (a, b, c) in distances:
            # print(a, b, c)
            matrix[a][b] = c
            matrix[b][a] = c
        return matrix

    adj_matrix = adjMatrix(cities, distances)
    h_n = 0
    route.append(route[0])
    length_route = len(route)
    for i in range(length_route - 1):
        x = route[i]
        y = route[i + 1]
        h_n += (30 - adj_matrix[x][y])

    return h_n


def hill_climbing(cities: int, distances: List[Tuple[int]]):
    solution = []
    curr_route = []
    # print(route)
    route_cost = 999
    random_route = transition(list(range(cities)))
    # print(random_route)
    for i in range(cities - 1):
        curr_route_1 = random_route[i]
        # print(curr_route_1)
        curr_route_2 = random_route[i + 1]
        # print(curr_route_2)
        curr_cost_1 = heuristic(cities, distances, curr_route_1)
        # print(curr_cost_1)
        curr_cost_2 = heuristic(cities, distances, curr_route_2)
        # print(curr_cost_2)
        if curr_cost_1 <= curr_cost_2:
            if route_cost > curr_cost_1:
                curr_route = curr_route_1
                # print(curr_route)
                route_cost = curr_cost_1
                # print(route_cost)
        else:
            if route_cost > curr_cost_2:
                curr_route = curr_route_2
                # print(curr_route)
                route_cost = curr_cost_2
                # print(route_cost)
    # print(curr_route)
    for i in range(cities):
        solution.append(curr_route[i])
    # print(solution)
    return solution


def hill_climbing_with_random_restarts(cities: int, distances: List[Tuple[int]], repeats: int = 10):
    route = []
    best_route = []
    best_cost = 999
    for i in range(repeats):
        curr_route = hill_climbing(cities, distances)
        curr_h_n = heuristic(cities, distances, curr_route)
        if (curr_h_n < best_cost):
            best_route = curr_route
            best_cost = curr_h_n
    for i in range(cities):
        route.append(best_route[i])
    # print(route)
    return route

# Test
@wrap_test
def test_random_restarts(cities: int, distances: List[Tuple[int]], repeats: int = 10):
    start = time.time()
    route = hill_climbing_with_random_restarts(cities, distances, repeats)
    print(f"Time lapsed: {time.time() - start}")

    assert sorted(route) == list(range(cities)), "Invalid route"

    return "PASSED"

if __name__ == '__main__':
    
    cities_1 = 4
    distances_1 = [(1, 0, 10), (0, 3, 22), (2, 1, 8), (2, 3, 30), (1, 3, 25), (0, 2, 15)]

    cities_2 = 10
    distances_2 = [(2, 7, 60), (1, 6, 20), (5, 4, 70), (9, 8, 90), (3, 7, 54), (2, 5, 61),
        (4, 1, 106), (0, 6, 51), (3, 1, 45), (0, 5, 86), (9, 2, 73), (8, 4, 14), (0, 1, 51),
        (9, 7, 22), (3, 2, 22), (8, 1, 120), (5, 7, 92), (5, 6, 60), (6, 2, 10), (8, 3, 78),
        (9, 6, 82), (0, 2, 41), (2, 8, 99), (7, 8, 71), (0, 9, 32), (4, 0, 73), (0, 3, 42),
        (9, 1, 80), (4, 2, 85), (5, 9, 113), (3, 6, 28), (5, 8, 81), (3, 9, 72), (9, 4, 81),
        (5, 3, 45), (7, 4, 60), (6, 8, 106), (0, 8, 85), (4, 6, 92), (7, 6, 70), (7, 0, 22),
        (7, 1, 73), (4, 3, 64), (5, 1, 80), (2, 1, 22)]

    print('Task 2.7:')
    print('cities_1: ' + test_random_restarts(cities_1, distances_1))
    print('cities_2: ' + test_random_restarts(cities_2, distances_2, 20))


#TODO Task 2.8: Compare local search with other search algorithms
