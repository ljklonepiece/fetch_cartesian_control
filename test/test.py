import rospy
from cartesian_server.srv import *

CARTESIAN_SERVER = 'fetch_cartesian'

class Test(object):

    def __init__(self):
        ## cartesian controller client
        self._cartesian_client = rospy.ServiceProxy(CARTESIAN_SERVER, CartesianGoal)
        rospy.wait_for_service(CARTESIAN_SERVER)

    def move_in_cartesian(self, dx=0, dy=0, dz=0):
        req = CartesianGoalRequest()
        req.dx = dx
        req.dy = dy
        req.dz = dz

        res = CartesianGoalResponse()
        try:
            rospy.wait_for_service(CARTESIAN_SERVER)
            res = self._cartesian_client(req)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr('Service call failed')
            return False, None, None

        return res.executed, res.pose_init, res.pose_final

if __name__ == '__main__':
    rospy.init_node('test')
    t = Test()
    print t.move_in_cartesian(dx=1, dy=1)
