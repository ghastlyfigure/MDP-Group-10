import itertools
import math
from collections import deque
from typing import Tuple

from algorithm import settings
from algorithm.entities.commands.scan_command import ScanCommand
from algorithm.entities.commands.straight_command import StraightCommand
from algorithm.entities.grid.obstacle import Obstacle
from algorithm.entities.robot.brain.mod_a_star import ModifiedAStar

import concurrent.futures

class Brain:
    def __init__(self, robot, grid):
        self.robot = robot
        self.grid = grid

        # Compute the simple Hamiltonian path for all obstacles
        # self.simple_hamiltonian = tuple()

        # replace simple_hamiltonian with every possible path
        self.simple_hamiltonian = []
        # Create all the commands required to finish the course.
        self.commands = deque()

    # def compute_simple_hamiltonian_path(self) -> Tuple[Obstacle]:
    def compute_simple_hamiltonian_path(self):
        """
        Get the Hamiltonian Path to all points with the best possible effort.
        This is a simple calculation where we assume that we travel directly to the next obstacle.
        """
        # Generate all possible permutations for the image obstacles
        perms = list(itertools.permutations(self.grid.obstacles))

        # Get the path that has the least distance travelled.
        def calc_distance(path):
            # Create all target points, including the start.
            targets = [self.robot.pos.xy_pygame()]
            for obstacle in path:
                targets.append(obstacle.pos.xy_pygame())

            dist = 0
            for i in range(len(targets) - 1):
                # dist += math.sqrt(((targets[i][0] - targets[i + 1][0]) ** 2) +
                #                   ((targets[i][1] - targets[i + 1][1]) ** 2))
                dist += abs(targets[i][0] - targets[i + 1][0]) + abs(targets[i][1] - targets[i + 1][1])
            return dist

        # simple = min(perms, key=calc_distance)
        perms.sort(key=calc_distance);
        simple = perms[0];
        print(simple)

        print("Found a simple hamiltonian path:")
        for ob in simple:
            print(f"\t{ob}")
        return perms

    def compress_paths(self):
        """
        Compress similar commands into one command.

        Helps to reduce the number of commands.
        """
        print("Compressing commands... ", end="")
        index = 0
        new_commands = deque()
        while index < len(self.commands):
            command = self.commands[index]
            if isinstance(command, StraightCommand):
                new_length = 0
                while index < len(self.commands) and isinstance(self.commands[index], StraightCommand):
                    new_length += self.commands[index].dist
                    index += 1
                command = StraightCommand(new_length)
                new_commands.append(command)
            else:
                new_commands.append(command)
                index += 1
        self.commands = new_commands
        print("Done!")

    def plan_path(self):
        print("-" * 40)
        print("STARTING PATH COMPUTATION...")
        # self.simple_hamiltonian = self.compute_simple_hamiltonian_path()
        if len(self.grid.obstacles) == 4:
            consider = 20
        elif len(self.grid.obstacles) > 4:
            consider = 100
        paths = self.compute_simple_hamiltonian_path()[0:consider]
        # self.simple_hamiltonian = paths[0]
        print()
        orders = []

        def calc_actual_distance(path):

            string_commands = [command.convert_to_message() for command in self.commands]
            total_dist = 0
            for command in string_commands:
                parts = command.split(",")
                if parts[0] == "1":  # straight
                    total_dist += int(parts[2])
                if parts[0] == "0":  # turn
                    total_dist += int(100)
            string_commands.append("finish")
            return total_dist

        index = 0
        for path in paths:
            self.simple_hamiltonian = path
        # for path in self.simple_hamiltonian:  # new!!
            self.commands.clear()
            order = []

            curr = self.robot.pos.copy()  # We use a copy rather than get a reference.
            # for obstacle in self.simple_hamiltonian:
            for obstacle in self.simple_hamiltonian:
                target = obstacle.get_robot_target_pos()
                # print(f"Planning {curr} to {target}")
                res = ModifiedAStar(self.grid, self, curr, target).start_astar()
                if res is None:
                    print(f"\tNo path found from {curr} to {obstacle}")
                else:
                    # print("\tPath found.")
                    curr = res
                    self.commands.append(ScanCommand(settings.ROBOT_SCAN_TIME, obstacle.index))
                    order.append(obstacle.index)
            orders.append((order, index, calc_actual_distance(paths[index])))
            index += 1

        def dist_key(list):
            return list[2]
        orders.sort(key=dist_key)

        best_index = orders[0][1]
        self.simple_hamiltonian = paths[best_index]
        self.commands.clear()

        curr = self.robot.pos.copy()  # We use a copy rather than get a reference.
        # for obstacle in self.simple_hamiltonian:
        for obstacle in self.simple_hamiltonian:
            target = obstacle.get_robot_target_pos()
            # print(f"Planning {curr} to {target}")
            res = ModifiedAStar(self.grid, self, curr, target).start_astar()
            if res is None:
                print(f"\tNo path found from {curr} to {obstacle}")
            else:
                # print("\tPath found.")
                curr = res
                self.commands.append(ScanCommand(settings.ROBOT_SCAN_TIME, obstacle.index))

        self.compress_paths()
        print("-" * 40)
        return orders[0][0]


    #
    # def plan_path(self):
    #     print("-" * 40)
    #     print("STARTING PATH COMPUTATION...")
    #     # self.simple_hamiltonian = self.compute_simple_hamiltonian_path()
    #     paths = self.compute_simple_hamiltonian_path()
    #     # self.simple_hamiltonian = paths[3]
    #     print()
    #     return_orders = []
    #     min_index = -1
    #     min = 100000
    #     index = 0
    #
    #     def calc_actual_distance(path):
    #         string_commands = [command.convert_to_message() for command in self.commands]
    #         total_dist = 0
    #         for command in string_commands:
    #             # print(command)
    #             parts = command.split(",")
    #             if parts[0] == "1":  # straight
    #                 total_dist += int(parts[2])
    #                 # print(int(parts[2]))
    #             if parts[0] == "0":  # turn
    #                 total_dist += int(100)
    #                 # print("turn, 100")
    #         # print(string_commands)
    #         string_commands.append("finish")
    #         # print("Done!")
    #         # print("total_dist = ", total_dist)
    #         return total_dist
    #
    #     for path in paths[0:1]:  # new!!
    #         order = []
    #         self.commands = deque()
    #
    #         curr = self.robot.pos.copy()  # We use a copy rather than get a reference.
    #         # for obstacle in self.simple_hamiltonian:
    #         for obstacle in path:
    #             target = obstacle.get_robot_target_pos()
    #             print(f"Planning {curr} to {target}")
    #             res = ModifiedAStar(self.grid, self, curr, target).start_astar()
    #             if res is None:
    #                 print(f"\tNo path found from {curr} to {obstacle}")
    #             else:
    #                 print("\tPath found.")
    #                 curr = res
    #                 self.commands.append(ScanCommand(settings.ROBOT_SCAN_TIME, obstacle.index))
    #                 order.append(obstacle.index)
    #         self.compress_paths()
    #         return_orders.append((order, index))
    #         index += 1
    #     return_orders.sort(key=calc_actual_distance)
    #     print(return_orders)
    #     print("-" * 40)
    #     self.commands = deque()
    #
    #     print(paths[return_orders[0][1]])
    #     for obstacle in paths[return_orders[0][1]]:
    #         target = obstacle.get_robot_target_pos()
    #         print(f"Planning {curr} to {target}")
    #         res = ModifiedAStar(self.grid, self, curr, target).start_astar()
    #         if res is None:
    #             print(f"\tNo path found from {curr} to {obstacle}")
    #         else:
    #             print("\tPath found.")
    #             curr = res
    #             self.commands.append(ScanCommand(settings.ROBOT_SCAN_TIME, obstacle.index))
    #             order.append(obstacle.index)
    #     self.compress_paths()
    #     return return_orders[0]
    #
    #

    # def plan_path(self):
    #     print("-" * 40)
    #     print("STARTING PATH COMPUTATION...")
    #     self.simple_hamiltonian = self.compute_simple_hamiltonian_path()
    #     print()
    #
    #     curr = self.robot.pos.copy()  # We use a copy rather than get a reference.
    #     with concurrent.futures.ThreadPoolExecutor() as executor:
    #         futures = []
    #         counter = 0
    #         for obstacle in self.simple_hamiltonian:
    #             if counter == 2:
    #                 break
    #             target = obstacle.get_robot_target_pos()
    #             print(f"Planning {curr} to {target}")
    #             future = executor.submit(ModifiedAStar(self.grid, self, curr, target).start_astar)
    #             futures.append(future)
    #             curr = target
    #             counter += 1
    #
    #         for i, future in enumerate(concurrent.futures.as_completed(futures)):
    #             obstacle = self.simple_hamiltonian[i]
    #             if future.result() is None:
    #                 print(f"\tNo path found from {curr} to {obstacle}")
    #             else:
    #                 print("\tPath found.")
    #                 curr = future.result()
    #                 self.commands.append(ScanCommand(settings.ROBOT_SCAN_TIME, obstacle.index))
    #
    #     self.compress_paths()
    #     print("-" * 40)
