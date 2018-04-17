import os
import rospy
import rospkg

from std_msgs.msg import Float32

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

# Button stylesheets
style_normal = """
color: black;
font-size: 26px;
"""
style_active = """
color: green;
font-size: 26px;
"""

class apm_modecontrol(Plugin):

    def __init__(self, context):
        super(apm_modecontrol, self).__init__(context)

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_apm_modecontrol'), 'resource', 'APM_modecontrol.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        # self._widget.setObjectName('SpeedometerUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Init ROS topic subscriber
        #self.speed_sub = rospy.Subscriber('cart_odom_speed', Float32, self._on_speed_update)

        # The current mode for the cart
        # 0 - Manual
        # 1 - Remote
        # 2 - Autonomous
        self.mode = 0;

        # Setup buttons
        self._widget.button_manual.clicked.connect(self.button_manual_pressed)
        self._widget.button_remote.clicked.connect(self.button_remote_pressed)
        self._widget.button_auto.clicked.connect(self.button_auto_pressed)
        self._update_button_state()

        # Timer for sending out the current mode
        self.publisher_mode = rospy.Publisher('apm_controlmode', Float32, queue_size=1)
        # rospy.Timer(rospy.Duration(1), self._publish_mode)

    def shutdown_plugin(self):
        #self.publisher_mode.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def button_manual_pressed(self):
        self.mode = 0
        self._update_button_state()
        self._publish_mode(None)

    def button_remote_pressed(self):
        self.mode = 1
        self._update_button_state()
        self._publish_mode(None)

    def button_auto_pressed(self):
        self.mode = 2
        self._update_button_state()
        self._publish_mode(None)

    # Updates the button UI state
    def _update_button_state(self):
        if self.mode == 0:
            self._widget.button_manual.setStyleSheet(style_active)
            self._widget.button_remote.setStyleSheet(style_normal)
            self._widget.button_auto.setStyleSheet(style_normal)
        elif self.mode == 1:
            self._widget.button_manual.setStyleSheet(style_normal)
            self._widget.button_remote.setStyleSheet(style_active)
            self._widget.button_auto.setStyleSheet(style_normal)
        else:
            self._widget.button_manual.setStyleSheet(style_normal)
            self._widget.button_remote.setStyleSheet(style_normal)
            self._widget.button_auto.setStyleSheet(style_active)

    # Publish the current mode of the cart
    def _publish_mode(self, event):
        self.publisher_mode.publish(Float32(self.mode))

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
