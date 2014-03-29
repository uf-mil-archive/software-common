#ifndef GUARD_GNBUTEVNTUFMKMYT
#define GUARD_GNBUTEVNTUFMKMYT

#include <Eigen/Dense>

namespace object_finder {
namespace fft {

void calc_pcc(Eigen::ArrayXXd const & image,
              //Eigen::ArrayXXd const & image_weight,
              Eigen::ArrayXXd const & template_,
              Eigen::ArrayXXd const & template_weight,
              int res_top,
              int res_left,
              unsigned int res_height,
              unsigned int res_width,
              Eigen::ArrayXXd * res);

}
}

#endif
