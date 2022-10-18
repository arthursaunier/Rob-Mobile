import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
goal = True

def movebase_client():
    
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    #pointA
    goal_1 = MoveBaseGoal()
    goal_1.target_pose.header.frame_id = "map"
    goal_1.target_pose.header.stamp = rospy.Time.now()
   
    goal_1.target_pose.pose.position.x = -0.1222
    goal_1.target_pose.pose.position.y = -5.526
   
    goal_1.target_pose.pose.orientation.z = 0.0
    goal_1.target_pose.pose.orientation.w = 1.0

    #pointB
    goal_2 = MoveBaseGoal()
    goal_2.target_pose.header.frame_id = "map"
    goal_2.target_pose.header.stamp = rospy.Time.now()
   
    goal_2.target_pose.pose.position.x = 6.45
    goal_2.target_pose.pose.position.y = -5.526
   
    goal_2.target_pose.pose.orientation.z = 1.0
    goal_2.target_pose.pose.orientation.w = 0.0

   # Sends the goal to the action server.
    if (goal):
        client.send_goal(goal_1)
        goal = False
    else:
        client.send_goal(goal_2)
        goal = True
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        while(1):
            result = movebase_client()
            if result:
                rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")