import os
import rospy
import rospkg

from std_msgs.msg import Float32
from std_msgs.msg import String

from PyQt4.QtGui import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
#from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionGoal

# Button stylesheets
style_normal = """
color: black;
font-size: 26px;"""

style_active = """
color: green;
font-size: 26px;"""


class apm_ui(Plugin):

    def __init__(self, context):
        super(apm_ui, self).__init__(context)

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
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_apm_ui'), 'resource', 'APM_UI.ui')
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
        self.mode = 0
        # Current Destination for the cart
        self.frame_id = '/map' #What does this need to be?
        self.msg = MoveBaseActionGoal()
        
        # Power voltage set maximum
        self._widget.V12_bar.setMaximum(10000)
        self._widget.V48_bar.setMaximum(10000)
        
        # Display a test image %%TODO: Update Later for actual used
        # self._widget.mapView = QLabel()
        #QPixmap default_image = QtGui.QPixmap(os.getcwd()+"/default_image.jpg")
        #image_location = os.path.join(rospkg.RosPack().get_path('rqt_apm_ui'), 'resource', 'default_image.png')
        #pixmap = QPixmap(image_location)
        #self._widget.mapView.setPixmap(pixmap)
        
        # Setup buttons
        self._widget.button_manual.clicked.connect(self.button_manual_pressed)
        self._widget.button_remote.clicked.connect(self.button_remote_pressed)
        self._widget.button_auto.clicked.connect(self.button_auto_pressed)
        self._widget.button_locationA.clicked.connect(self.button_locationA_pressed)
        self._widget.button_locationB.clicked.connect(self.button_locationB_pressed)
        self._widget.button_locationC.clicked.connect(self.button_locationC_pressed)
        self._widget.button_locationD.clicked.connect(self.button_locationD_pressed)
        self._widget.button_locationE.clicked.connect(self.button_locationE_pressed)
        self._widget.button_locationF.clicked.connect(self.button_locationF_pressed)
        self._widget.button_locationG.clicked.connect(self.button_locationG_pressed)
        self._update_button_state()

        # Timer for sending out the current mode
        self.publisher_mode = rospy.Publisher('apm_controlmode', Float32, queue_size=1)
        self.publisher_destination = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
        rospy.loginfo("Up to Subscribers")
        self.subscriber_mode = rospy.Subscriber('revert_manual', Float32, self.e_controlmode)
        self.subscriber_12V = rospy.Subscriber('apm_12V_power', Float32, self.V12_callback, queue_size=1)
        self.subscriber_48V = rospy.Subscriber('apm_48V_power', Float32, self.V48_callback)
        rospy.loginfo("Subscribers Created")
        #rospy.Timer(rospy.Duration(1), self._publish_mode)

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
        
    def button_locationA_pressed(self):
        self.msg = MoveBaseActionGoal();
        #header
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.frame_id
        self.msg.header.seq = 0
        #GoalID
        self.msg.goal_id.stamp = rospy.Time.now()
        self.msg.goal_id.id = 'A'
        #MoveBaseGoal
        #header
        self.msg.goal.target_pose.header.stamp = rospy.Time.now()
        self.msg.goal.target_pose.header.frame_id = self.frame_id
        self.msg.goal.target_pose.header.seq = 0
        #pose
        self.msg.goal.target_pose.pose.position.x = 3.881
        self.msg.goal.target_pose.pose.position.y = 20.942
        self.msg.goal.target_pose.pose.position.z = 0
        #quaternian
        self.msg.goal.target_pose.pose.orientation.x = 0
        self.msg.goal.target_pose.pose.orientation.y = 0
        self.msg.goal.target_pose.pose.orientation.z = 0.994
        self.msg.goal.target_pose.pose.orientation.w = -0.110
        
        self._update_button_state()
        self._publish_path(None)
        
    def button_locationB_pressed(self):  

        self.msg = MoveBaseActionGoal();
        #header
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.frame_id
        self.msg.header.seq = 0
        #GoalID
        self.msg.goal_id.stamp = rospy.Time.now()
        self.msg.goal_id.id = 'B'
        #MoveBaseGoal
        #header
        self.msg.goal.target_pose.header.stamp = rospy.Time.now()
        self.msg.goal.target_pose.header.frame_id = self.frame_id
        self.msg.goal.target_pose.header.seq = 0
        #pose
        self.msg.goal.target_pose.pose.position.x = -8.262
        self.msg.goal.target_pose.pose.position.y = 0.750
        self.msg.goal.target_pose.pose.position.z = 0
        #quaternian
        self.msg.goal.target_pose.pose.orientation.x = 0
        self.msg.goal.target_pose.pose.orientation.y = 0
        self.msg.goal.target_pose.pose.orientation.z = -0.469
        self.msg.goal.target_pose.pose.orientation.w = 0.883
        
        self._update_button_state()
        self._publish_path(None)

        
    def button_locationC_pressed(self):
        self.msg = MoveBaseActionGoal();
        #header
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.frame_id
        self.msg.header.seq = 0
        #GoalID
        self.msg.goal_id.stamp = rospy.Time.now()
        self.msg.goal_id.id = 'C'
        #MoveBaseGoal
        #header
        self.msg.goal.target_pose.header.stamp = rospy.Time.now()
        self.msg.goal.target_pose.header.frame_id = self.frame_id
        self.msg.goal.target_pose.header.seq = 0
        #pose
        self.msg.goal.target_pose.pose.position.x = 14.333
        self.msg.goal.target_pose.pose.position.y = 1.512
        self.msg.goal.target_pose.pose.position.z = 0
        #quaternian
        self.msg.goal.target_pose.pose.orientation.x = 0
        self.msg.goal.target_pose.pose.orientation.y = 0
        self.msg.goal.target_pose.pose.orientation.z = 0.513
        self.msg.goal.target_pose.pose.orientation.w = 0.859
        
        self._update_button_state()
        self._publish_path(None)

        
    def button_locationD_pressed(self):
        self.msg = MoveBaseActionGoal();
        #header
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.frame_id
        self.msg.header.seq = 0
        #GoalID
        self.msg.goal_id.stamp = rospy.Time.now()
        self.msg.goal_id.id = 'D'
        #MoveBaseGoal
        #header
        self.msg.goal.target_pose.header.stamp = rospy.Time.now()
        self.msg.goal.target_pose.header.frame_id = self.frame_id
        self.msg.goal.target_pose.header.seq = 0
        #pose
        self.msg.goal.target_pose.pose.position.x = -16.624
        self.msg.goal.target_pose.pose.position.y = -17.404
        self.msg.goal.target_pose.pose.position.z = 0
        #quaternian
        self.msg.goal.target_pose.pose.orientation.x = 0
        self.msg.goal.target_pose.pose.orientation.y = 0
        self.msg.goal.target_pose.pose.orientation.z = -0.673
        self.msg.goal.target_pose.pose.orientation.w = 0.740
        
        self._update_button_state()
        self._publish_path(None)

    def button_locationE_pressed(self):
        self.msg = MoveBaseActionGoal();
        #header
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.frame_id
        self.msg.header.seq = 0
        #GoalID
        self.msg.goal_id.stamp = rospy.Time.now()
        self.msg.goal_id.id = 'E'
        #MoveBaseGoal
        #header
        self.msg.goal.target_pose.header.stamp = rospy.Time.now()
        self.msg.goal.target_pose.header.frame_id = self.frame_id
        self.msg.goal.target_pose.header.seq = 0
        #pose
        self.msg.goal.target_pose.pose.position.x = -17.042
        self.msg.goal.target_pose.pose.position.y = -33.911
        self.msg.goal.target_pose.pose.position.z = 0
        #quaternian
        self.msg.goal.target_pose.pose.orientation.x = 0
        self.msg.goal.target_pose.pose.orientation.y = 0
        self.msg.goal.target_pose.pose.orientation.z = -0.783
        self.msg.goal.target_pose.pose.orientation.w = 0.622
        
        self._update_button_state()
        self._publish_path(None)

        
    def button_locationF_pressed(self):
        self.msg = MoveBaseActionGoal();
        #header
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.frame_id
        self.msg.header.seq = 0
        #GoalID
        self.msg.goal_id.stamp = rospy.Time.now()
        self.msg.goal_id.id = 'F'
        #MoveBaseGoal
        #header
        self.msg.goal.target_pose.header.stamp = rospy.Time.now()
        self.msg.goal.target_pose.header.frame_id = self.frame_id
        self.msg.goal.target_pose.header.seq = 0
        #pose
        self.msg.goal.target_pose.pose.position.x = -25.136
        self.msg.goal.target_pose.pose.position.y = -22.894
        self.msg.goal.target_pose.pose.position.z = 0
        #quaternian
        self.msg.goal.target_pose.pose.orientation.x = 0
        self.msg.goal.target_pose.pose.orientation.y = 0
        self.msg.goal.target_pose.pose.orientation.z = -0.803
        self.msg.goal.target_pose.pose.orientation.w = 0.596
        
        self._update_button_state()
        self._publish_path(None)

        
    def button_locationG_pressed(self):
        self.msg = MoveBaseActionGoal();
        #header
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.frame_id
        self.msg.header.seq = 0
        #GoalID
        self.msg.goal_id.stamp = rospy.Time.now()
        self.msg.goal_id.id = 'G'
        #MoveBaseGoal
        #header
        self.msg.goal.target_pose.header.stamp = rospy.Time.now()
        self.msg.goal.target_pose.header.frame_id = self.frame_id
        self.msg.goal.target_pose.header.seq = 0
        #pose
        self.msg.goal.target_pose.pose.position.x = -23.348
        self.msg.goal.target_pose.pose.position.y = -44.090
        self.msg.goal.target_pose.pose.position.z = 0
        #quaternian
        self.msg.goal.target_pose.pose.orientation.x = 0
        self.msg.goal.target_pose.pose.orientation.y = 0
        self.msg.goal.target_pose.pose.orientation.z = -0.815
        self.msg.goal.target_pose.pose.orientation.w = 0.580
        
        self._update_button_state()
        self._publish_path(None)


    # Updates the UI state
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
        #if self.destination == 0:
        #    self._widget.button_locationA.setStyleSheet(style_normal)
        #    self._widget.button_locationB.setStyleSheet(style_normal)
        #    self._widget.button_locationC.setStyleSheet(style_normal)
        #    self._widget.button_locationD.setStyleSheet(style_normal)
        #    self._widget.button_locationE.setStyleSheet(style_normal)
        #    self._widget.button_locationF.setStyleSheet(style_normal)
        #    self._widget.button_locationG.setStyleSheet(style_normal)
        #else:
        #    self._widget.button_locationA.setStyleSheet(style_active)
        #    self._widget.button_locationB.setStyleSheet(style_active)
        #    self._widget.button_locationC.setStyleSheet(style_active)
        #    self._widget.button_locationD.setStyleSheet(style_active)
        #    self._widget.button_locationE.setStyleSheet(style_active)
        #    self._widget.button_locationF.setStyleSheet(style_active)
        #    self._widget.button_locationG.setStyleSheet(style_active)
        #self._widget.V12_bar.setValue((self.V12/12)*100)
        #self._widget.V48_bar.setValue((self.V48/48)*100)
        
    # Publish the current mode of the cart
    def _publish_mode(self, event):
        self.publisher_mode.publish(Float32(self.mode))
        
    def _publish_path(self, event):
        self.publisher_destination.publish(self.msg)
        
    def subscriber_manual(data):
        self.mode = int(data)
    
    def e_controlmode(self, data):
        rospy.loginfo("Callback")
        self.mode = 0
        self._widget.button_manual.setStyleSheet(style_active)
        self._widget.button_remote.setStyleSheet(style_normal)
        self._widget.button_auto.setStyleSheet(style_normal)
        self.publisher_mode.publish(Float32(self.mode))
        # rospy.loginfo("Callback Done")

    def V12_callback(self, data):
        self._widget.V12_bar.setValue(int(round(data.data*100)))
    
    def V48_callback(self, data):
        self._widget.V48_bar.setValue(int(round(data.data*100)))
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
