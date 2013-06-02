#ifndef SPHERE_FINDING_H
#define SPHERE_FINDING_H

#include <Eigen/Dense>

#include "renderbuffer.h"

void sphere_draw(RenderBuffer &renderbuffer, RenderBuffer::RegionType region, Eigen::Vector3d pos, double radius);

#endif
