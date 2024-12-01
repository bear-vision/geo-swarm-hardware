import py_trees
import py_trees_ros
from custom_srv_interfaces.msg import PerceptionStuff

def perception_to_blackboard():
    """Creates a blackboard to view ENTIRE PerceptionStuff message. 
       ROS subscribers are asynchronous, while py-trees are synchronous. This means 
       that sometimes there will be no messages on a tick, or there will be new messages
       when there are no ticks. Blackboards addresses this issue:
            1. It saves the latest message received from the subscribed topic to the blackboard.
            2. If no data has been received yet, it returns a RUNNING status.
            3. Once data is received and saved to the blackboard, it returns a SUCCESS status.

    Returns:
        py_trees_ros.subscribers.ToBlackboard: the blackboard behaviour containing the latest PerceptionStuff message
    """
    perception2BB =  py_trees_ros.subscribers.ToBlackboard(
        name="LocalPositionToBlackboard",
        topic_name="realsense/out/perception_stuff",
        topic_type=PerceptionStuff,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched() # we care about most recent data, no need to ensure all messages were delivered
    )
    
    return perception2BB