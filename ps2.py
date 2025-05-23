""" CS2109S Problem Set 2: Informed Search"""

import copy
import heapq
import math
import random
import time
from typing import List, Tuple

import cube
import utils

""" ADD HELPER FUNCTION HERE """

"""
We provide implementations for the Node and PriorityQueue classes in utils.py, but you can implement your own if you wish
"""
from utils import Node
from utils import PriorityQueue


#TODO Task 1.1: Implement your heuristic function, which takes in an instance of the Cube and
#   the State class and returns the estimated cost of reaching the goal state from this state.
def heuristic_func(problem: cube.Cube, state: cube.State) -> float:
    r"""
    Computes the heuristic value of a state
    
    Args:
        problem (cube.Cube): the problem to compute
        state (cube.State): the state to be evaluated

    Returns:
        h_n (float): the heuristic value 
    """
    h_n = 0.0
    goals = problem.goal

    """ YOUR CODE HERE """

    """ END YOUR CODE HERE """

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
    
    """ YOUR CODE HERE """
 
    """ END YOUR CODE HERE """
    
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
    r"""
    Generates new routes to be used in the next iteration in the hill-climbing algorithm.

    Args:        
        route (List[int]): The current route as a list of cities in the order of travel

    Returns:
        new_routes (List[List[int]]): New routes to be considered
    """
    new_routes = []
    
    """ YOUR CODE HERE """
    
    """ END YOUR CODE HERE """

    return new_routes

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
def heuristic(cities: int, distances: List[Tuple[int]], route: List[int]):
    r"""
    Computes the heuristic value of a route

    Args:
        cities (int): The number of cities to be visited

        distances (List[Tuple[int]]): The list of distances between every two cities
            Each distance is represented as a tuple in the form of (c1, c2, d), where
                c1 and c2 are the two cities and d is the distance between them.
            The length of the list should be equal to cities * (cities - 1)/2.

        route (List[int]): The current route as a list of cities in the order of travel

    Returns:
        h_n: the heuristic value
    """
    h_n = 0
    
    """ YOUR CODE HERE """
    
    """ END YOUR CODE HERE """

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
def hill_climbing(cities: int, distances: List[Tuple[int]]):
    r"""
    Hill climbing finds the solution to reach the goal from the initial.
    
    Args:
        cities (int): The number of cities to be visited
        
        distances (List[Tuple[int]]): The list of distances between every two cities
            Each distance is represented as a tuple in the form of (c1, c2, d), where
                c1 and c2 are the two cities and d is the distance between them.
            The length of the list should be equal to cities * (cities - 1)/2.

    Returns:
        route (List[int]): The shortest route, represented by a list of cities
            in the order to be traversed.
    """

    route = []
    
    """ YOUR CODE HERE """

    """ END YOUR CODE HERE """
    
    return route

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
def hill_climbing_with_random_restarts(cities: int, distances: List[Tuple[int]], repeats: int = 10):
    r"""
    Hill climbing with random restarts finds the solution to reach the goal from the initial.
    
    Args:
        cities (int): The number of cities to be visited
        
        distances (List[Tuple[int]]): The list of distances between every two cities
            Each distance is represented as a tuple in the form of (c1, c2, d), where
                c1 and c2 are the two cities and d is the distance between them.
            The length of the list should be equal to cities * (cities - 1)/2.

        repeats (int): The number of times hill climbing to be repeated. The default
            value is 10.

    Returns:
        route (List[int]): The shortest route, represented by a list of cities
            in the order to be traversed.
    """

    route = []
    
    """ YOUR CODE HERE """

    """ END YOUR CODE HERE """
    
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
