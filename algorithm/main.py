import sys
import time
from typing import List

from algorithm import settings
from algorithm.app import AlgoSimulator, AlgoMinimal
from algorithm.entities.assets.direction import Direction
from algorithm.entities.connection.rpi_client import RPiClient
from algorithm.entities.connection.rpi_server import RPiServer
from algorithm.entities.grid.obstacle import Obstacle


def parse_obstacle_data(data) -> List[Obstacle]:
    obs = []
    for obstacle_params in data:
        obs.append(Obstacle(obstacle_params[0],
                            obstacle_params[1],
                            Direction(obstacle_params[2]),
                            obstacle_params[3]))
    # [[x, y, orient, index], [x, y, orient, index]]
    return obs


def run_simulator():
    # Fill in obstacle positions with respect to lower bottom left corner.
    # (x-coordinate, y-coordinate, Direction)
    # obstacles = [[105, 75, 90, 0], [175, 25, 180, 1], [175, 175, 180, 2], [75, 125, 180, 4], [15, 185, -90, 5]]
    obstacles = [[15, 95, -90, 1], [55, 35, 0, 2]]
    # obstacles = [[15, 95, -90, 0]]
    obs = parse_obstacle_data(obstacles)
    app = AlgoSimulator(obs)
    order = app.init()
    print(order)
    app.robot.convert_all_commands()
    app.execute()


def run_minimal(also_run_simulator):
    # Create a client to connect to the RPi.
    print(f"Attempting to connect to {settings.RPI_HOST}:{settings.RPI_PORT}")
    client = RPiClient(settings.RPI_HOST, settings.RPI_PORT)
    # Wait to connect to RPi.
    while True:
        try:
            client.connect()
            break
        except OSError:
            pass
        except KeyboardInterrupt:
            client.close()
            sys.exit(1)
    print("Connected to RPi!\n")

    print("Waiting to receive obstacle data from RPi...")
    # Create a server to receive information from the RPi.
    server = RPiServer(settings.PC_HOST, settings.PC_PORT)
    # Wait for the RPi to connect to the PC.
    try:
        server.start()
    except OSError or KeyboardInterrupt as e:
        print(e)
        server.close()
        client.close()
        sys.exit(1)

    # At this point, both the RPi and the PC are connected to each other.
    # Create a synchronous call to wait for RPi data.
    obstacle_data: list = server.receive_data()
    server.close()
    print("Got data from RPi:")
    print(obstacle_data)

    obstacles = parse_obstacle_data(obstacle_data)
    if also_run_simulator:
        app = AlgoSimulator(obstacles)
        app.init()
        app.execute()
    app = AlgoMinimal(obstacles)
    app.init()
    order = app.execute()

    # Send the list of commands over.
    print("Sending list of commands to RPi...")
    commands = app.robot.convert_all_commands()
    order_and_commands = [order, commands]
    client.send_message(order_and_commands)
    client.close()


def run_rpi():
    while True:
        run_minimal(False)
        time.sleep(5)


if __name__ == '__main__':
    run_simulator()
    # run_minimal(False)