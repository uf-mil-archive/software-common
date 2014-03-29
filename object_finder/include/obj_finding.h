#ifndef OBJ_FINDING_H
#define OBJ_FINDING_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "object_finder/Component.h"

#include "renderbuffer.h"

namespace object_finder {
namespace obj_finding {

void draw(Component const & component, RenderBuffer &renderbuffer, int region, Eigen::Vector3d pos, Eigen::Quaterniond orientation);

}
}

#endif
