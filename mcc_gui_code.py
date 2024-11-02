import rclpy
from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QLabel, QVBoxLayout, QWidget
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
import time


class MyRqtPlugin(Plugin):
    def __init__(self, context):
        super(MyRqtPlugin, self).__init__(context)
        self.setObjectName("MyRqtPlugin")

        # Create the main widget and layout
        self._widget = QWidget()
        layout = QVBoxLayout()
        self._widget.setLayout(layout)

        # Create labels to display various data
        self.time_label = QLabel("Current Time: N/A")
        self.position_label = QLabel("Robot Position: N/A")
        self.battery_label = QLabel("Battery Voltage: N/A")
        self.uptime_label = QLabel("Uptime: N/A")

        # Add the labels to the layout
        layout.addWidget(self.time_label)
        layout.addWidget(self.position_label)
        layout.addWidget(self.battery_label)
        layout.addWidget(self.uptime_label)

        # Initialize the ROS2 node
        rclpy.init()
        self.node = RqtNode(self)

        # Add the widget to the RQT context
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        """Shutdown the plugin and ROS2 node"""
        self.node.destroy_node()
        rclpy.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        """Save the plugin's settings when needed"""
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore the plugin's settings when needed"""
        pass


class RqtNode(Node):
    def __init__(self, gui_plugin):
        super().__init__("my_rqt_plugin_node")
        self.gui_plugin = gui_plugin

        # Subscribe to relevant topics
        self.create_subscription(String, "/current_time", self.time_callback, 10)
        self.create_subscription(Float32, "/robot/position", self.position_callback, 10)
        self.create_subscription(Float32, "/robot/battery_voltage", self.battery_callback, 10)

    # Callback functions to update the GUI
    def time_callback(self, msg):
        self.gui_plugin.time_label.setText(f"Current Time: {msg.data}")

    def position_callback(self, msg):
        self.gui_plugin.position_label.setText(f"Robot Position: {msg.data}")

    def battery_callback(self, msg):
        self.gui_plugin.battery_label.setText(f"Battery Voltage: {msg.data} V")
