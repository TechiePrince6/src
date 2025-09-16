import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    
    a, b = 7, 12
    future = node.send_request(a, b)
    
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info(f"Result: {future.result().sum}")
    else:
        node.get_logger().error("Service call failed")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
