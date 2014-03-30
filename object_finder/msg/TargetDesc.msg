uint32 TYPE_MESH=0
uint32 TYPE_SPHERE=1
uint32 type

Mesh mesh

float64 sphere_radius
Color sphere_color
Color sphere_background_color

geometry_msgs/PoseWithCovariance prior_distribution
float64 min_dist
float64 max_dist # prior_distribution.pose.position ignored if this is nonzero
bool check_180z_flip

bool disallow_yawing
bool allow_pitching
bool allow_rolling

#bool is_180z_symmetric

geometry_msgs/Point extract_origin
geometry_msgs/Vector3 extract_x
geometry_msgs/Vector3 extract_y
uint32 extract_x_pixels
uint32 extract_y_pixels
