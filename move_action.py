import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf.transformations



def move_to(goal_x, goal_y, goal_yaw):
    
    cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    # wait for the action server to establish connection
    cli.wait_for_server()

    # fill ROS message
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position = Point(goal_x, goal_y, 0)
    quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    pose.pose.orientation = Quaternion(*quat)

    goal = MoveBaseGoal()
    goal.target_pose = pose

    # send message to the action server
    cli.send_goal(goal)

    # wait for the action server to complete the order
    cli.wait_for_result()

    # print result of navigation
    action_state = cli.get_state()
    if action_state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Navigation Succeeded.")

if __name__=="__main__":
    import time
    rospy.init_node('test')
    # initialize action client
    print("go")
    move_to(1, 0, 0, )
    # time.sleep(3)
    print('back')
    move_to(0, 0, 0)
    