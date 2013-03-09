#ifndef SPHERE_FINDING_H
#define SPHERE_FINDING_H

#include <Eigen/Dense>

#include "image.h"

void sphere_query(const TaggedImage &image, Eigen::Vector3d pos, double radius, Eigen::Vector3d &total_color, double &count, std::vector<int>* dbg_image=NULL);

#endif
