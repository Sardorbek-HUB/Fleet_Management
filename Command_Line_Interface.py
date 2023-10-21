import click
import rclpy
from fleet.management.action import FleetManagement
from rclpy.action import ActionClient

# Define the CLI command group
@click.group()
def humble_cli():
    """
    Welcome to the Humble CLI for vehicle allocation and routing.
    """

# Defining the subcommand for allocating and routing vehicles
@humble_cli.command()
@click.option('--fleet-size', type=int, prompt=True, help='Specify the fleet size.')
def allocate_route(fleet_size):
    """
    Allocate and route vehicles for a given fleet size.
    """
    # Initializing ROS2 node
    rclpy.init()

    # Creating an Action Client for FleetManagement
    action_client = ActionClient('/fleet_management', FleetManagement)

    # Creating a goal for the Action Client
    goal_msg = FleetManagement.Goal()
    goal_msg.fleet_size = fleet_size

    # Sending the goal to the Action Server
    future = action_client.send_goal_async(goal_msg)

    # Waiting for the result
    rclpy.spin_until_future_complete(future)
    result = future.result()

    if result:
        # Processing and displaying the routes
        routes = result.result.vehicle_routes
        click.echo('Routes:')
        for i, route in enumerate(routes):
            click.echo(f'Vehicle {i + 1}: {route}')
    else:
        click.echo('Failed to allocate and route vehicles.')

    # Shutdown the ROS2 node
    rclpy.shutdown()

if __name__ == '__main__':
    humble_cli()

