# Location of the tower (we should only care about x, y, z)
geometry_msgs/Pose tower_position

# Identify dirty patches (assume paint blobs are circular)
bool detected_dirty_patch # True if at least one dirty patch at the current level
int num_dirty_patches
geometry_msgs/PoseArray dirty_patches # on the current level
float32[] dirty_patches_radii

