import rclpy
import click
from rclpy.action import ActionClient
from rclpy.node import Node
from your_package_name.action import FleetManagement

class FleetManagementClient(Node):
    def __init__(self):
        super().__init__('fleet_management_client')
        self.action_client = ActionClient(self, FleetManagement, 'fleet_management')

    def send_request(self, fleet_size):
        goal_msg = FleetManagement.Goal()
        goal_msg.fleet_size = fleet_size

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected. Exiting...')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result_msg = future.result().result
        self.get_logger().info('Routes received:')
        for i, route in enumerate(result_msg.vehicle_routes):
            self.get_logger().info(f'Vehicle {i + 1}: {route}')

@click.command()
@click.option('--fleet-size', default=1, help='Specify the fleet size.')
def main(fleet_size):
    rclpy.init()
    client = FleetManagementClient()
    client.send_request(fleet_size)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
