uint32 TYPE_OBJECT=0
uint32 TYPE_SPHERE=1
uint32 type

string object_filename

float64 sphere_radius

bool relative_to_first
geometry_msgs/PoseWithCovariance prior_distribution
float64 min_dist
float64 max_dist # prior_distribution.pose.position ignored if this is nonzero
bool check_180z_flip

bool disallow_yawing
bool allow_pitching
bool allow_rolling

std_msgs/ColorRGBA bg_color
std_msgs/ColorRGBA fg_color

bool is_180z_symmetric

geometry_msgs/Point extract_origin
geometry_msgs/Vector3 extract_x
geometry_msgs/Vector3 extract_y
uint32 extract_x_pixels
uint32 extract_y_pixels
