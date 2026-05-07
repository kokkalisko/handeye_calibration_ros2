from rclpy.node import Node
import rclpy
from std_msgs.msg import String, Int32
from opcua import Client
from opcua.ua.uaerrors import BadSessionIdInvalid
from ur_sorter_calibration.utils import words_to_int32

class OPCUAClient:
    def __init__(self, url):
        self.client = Client(url)
        self.client.connect()
        print("Connected to OPC UA server.")

    def disconnect(self):
        self.client.disconnect()
        print("Disconnected from OPC UA server.")

    def read_value(self, node_id):
        node = self.client.get_node(node_id)
        return node.get_value()

class EncoderReader(Node):
    '''Node to read encoder counts from an OPC UA server and publish keypress events for synchronization with the marker reader.'''
    def __init__(self):
        super().__init__('encoder_reader')

        self.declare_parameter('protocol', 'opcua')

        # OPC UA parameters
        self.declare_parameter('opcua_server_url', '')
        self.declare_parameter('opc_ua_node_id', '')

        # Modbus parameters (if needed in the future)
        self.declare_parameter('ip', '')
        self.declare_parameter('port', 502)

        self.opcua_server_url = self.get_parameter('opcua_server_url').get_parameter_value().string_value
        self.node_id = self.get_parameter('opc_ua_node_id').get_parameter_value().string_value

        # Initialize the OPC UA client
        self.init_opc_ua_client()

        # Create the subscriber for keypress events (and synchronize with marker reader)
        self.keypress_subn = self.create_subscription(String, 'keypress_topic', self.keypress_callback, 10)

        # Create the publisher for encoder counts (and synchronize with marker reader)
        self.encoder_publisher = self.create_publisher(Int32, 'encoder_count', 10)

        # Initialize encoder count
        self.enc_count = 0

    def init_opc_ua_client(self):
        """Initialize the OPC UA client."""
        self.get_logger().info(f"Connecting to OPC UA server at {self.opcua_server_url}")
        self.opc_ua_client = OPCUAClient(self.opcua_server_url)
        self.opc_ua_node_id = self.node_id

    def get_opc_ua_value(self):
        """ Get value from OPC UA server using the stored node ID """
        try:
            node = self.opc_ua_client.client.get_node(self.opc_ua_node_id)
            return node.get_value()
        except BadSessionIdInvalid:
            self.opc_ua_client.client.connect()
            node = self.opc_ua_client.client.get_node(self.opc_ua_node_id)
            return node.get_value()
        except Exception as e:
            self.get_logger().error(f"Error reading from OPC UA server: {str(e)}")
            return None
        
    def capture_enc_count(self):
        """ Get encoder count from OPC UA and update the GUI"""
        try:
            # Read the encoder count from the OPC UA server
            read_values = self.get_opc_ua_value()
            if read_values is not None:
                self.enc_count = words_to_int32(read_values[12], read_values[13])
            else:
                self.get_logger().error("Error: Encoder count data not found")
                
        except Exception as e:
            self.get_logger().error(f"Error reading Encoder count: {str(e)}")    

    def keypress_callback(self, msg):
        """Callback function for keypress events. Captures the encoder count and logs it."""
        self.get_logger().info(f"Received keypress event: {msg.data}")
        if msg.data == 'q':
            self.capture_enc_count()
            self.get_logger().info(f"Captured encoder count: {self.enc_count}")
            encoder_msg = Int32()
            encoder_msg.data = self.enc_count
            self.encoder_publisher.publish(encoder_msg)
        elif msg.data == 'e':
            self.get_logger().info("Ending program...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    encoder_reader = EncoderReader()

    rclpy.spin(encoder_reader)

    encoder_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
