import rospy
from geometry_msgs.msg import Point, Pose
from mavros_driver.message_tools import yaw_to_orientation

class TakeOff:
    def __init__(self, sm):
        cp = sm.mav.current_pose.pose.position
        p = Point(cp.x, cp.y, cp.z + rospy.get_param("starting_altitude", 1.0))
        o = yaw_to_orientation(0)
        home = Pose(p, o)
        sm.home = home
        sm.mav.set_target_pose(home)
        if rospy.get_param("takeoff_test", True):
            # make a plus sign (+) movement to test movement control
            d = rospy.get_param("takeoff_test_dist", 1.0)
            self.takeoff_coords = [
                Pose(Point(p.x + d, p.y, p.z), o),
                home,
                Pose(Point(p.x, p.y + d, p.z), o),
                home,
                Pose(Point(p.x - d, p.y, p.z), o),
                home,
                Pose(Point(p.x, p.y - d, p.z), o),
                home
            ]
        else:
            self.takeoff_coords = []

    def execute(self, sm):
        if not sm.mav.controllable():
            from states.wait_for_control import WaitForControl
            sm.set_state(WaitForControl(sm))
            return

        if (sm.mav.has_arrived()):
            if len(self.takeoff_coords) > 0:
                sm.mav.set_target_pose(self.takeoff_coords.pop(0))
            else:
                from states.center_target import CenterTarget
                sm.set_state(CenterTarget(sm))