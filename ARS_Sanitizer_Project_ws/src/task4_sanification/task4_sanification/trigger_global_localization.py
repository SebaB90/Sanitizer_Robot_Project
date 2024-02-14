import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class GlobalLocalizationClient(Node):

    def __init__(self):
        super().__init__('global_localization_client')
        self.client = self.create_client(Empty, 'reinitialize_global_localization')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Il servizio non è ancora disponibile, in attesa...')
        self.req = Empty.Request()

    def send_request(self):
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Richiesta di re-inizializzazione globale inviata!')
        except Exception as e:
            self.get_logger().error('Servizio di re-inizializzazione fallito: %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    global_localization_client = GlobalLocalizationClient()
    global_localization_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(global_localization_client)
        if global_localization_client.future.done():
            try:
                global_localization_client.future.result()
            except Exception as e:
                global_localization_client.get_logger().error(
                    'Chiamata al servizio fallita %r' % (e,))
            break

    global_localization_client.destroy_node()
    rclpy.shutdown()
