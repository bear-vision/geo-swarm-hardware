import py_trees
import py_trees_ros
from px4_msgs.msg import VehicleLocalPosition

def vehicle_local_position_to_blackboard():
    """Creates a blackboard to view ENTIRE VehicleLocalPosition message. 
       ROS subscribers are asynchronous, while py-trees are synchronous. This means 
       that sometimes there will be no messages on a tick, or there will be new messages
       when there are no ticks. Blackboards addresses this issue:
            1. It saves the latest message received from the subscribed topic to the blackboard.
            2. If no data has been received yet, it returns a RUNNING status.
            3. Once data is received and saved to the blackboard, it returns a SUCCESS status.

    Returns:
        py_trees_ros.subscribers.ToBlackboard: the blackboard behaviour containing the latest VehicleLocalPosition message
        
    Note:
        - The VehicleLocalPosition is in NED frame
    """
    localPosition2BB =  py_trees_ros.subscribers.ToBlackboard(
        name="LocalPositionToBlackboard",
        topic_name="/fmu/out/vehicle_local_position",
        topic_type=VehicleLocalPosition,
        blackboard_variables={
            'position/x': 'x',
            'position/y': 'y',
            'position/z': 'z',
            'orientation/yaw': 'heading',
            'valid/xy_valid': 'xy_valid',
            'valid/z_valid': 'z_valid'
        },
        initialise_variables={ # dummy defaults to ensure we can run the tree without complaining about missing blackboard args
            'position/x': 0.0,
            'position/y': 0.0,
            'position/z': 0.0,
            'orientation/yaw': 0.0,
            'valid/xy_valid': False,
            'valid/z_valid': False
        },
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched() # we care about most recent data, no need to ensure all messages were delivered
    )
    
    return localPosition2BB