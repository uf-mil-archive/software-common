#ifndef IMAGE_H
#define IMAGE_H

#include <vector>

#include <Eigen/Dense>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


struct TaggedImage {
    sensor_msgs::CameraInfo cam_info;
    Eigen::Matrix<double, 3, 4> proj;
    std::vector<Eigen::Vector3d> sumimage;
    Eigen::Affine3d transform;
    
    inline TaggedImage(const sensor_msgs::Image &image, const sensor_msgs::CameraInfo &cam_info, const Eigen::Affine3d &transform) :
        cam_info(cam_info), transform(transform) {
        
        for(int row = 0; row < 3; row++)
            for(int col = 0; col < 4; col++)
                proj(row, col) = cam_info.P[4*row + col];
        
        sumimage.resize(image.width * image.height);
        
        int step;
        if(image.encoding == "rgba8") step = 4;
        else if(image.encoding == "rgb8") step = 3;
        else assert(false);
        
        for(uint32_t row = 0; row < image.height; row++) {
            Eigen::Vector3d row_cumulative_sum(0, 0, 0);
            for(uint32_t col = 0; col < image.width; col++) {
                const uint8_t *pixel = image.data.data() + image.step * row + step * col;
                double r = pixel[0] / 255.;
                double g = pixel[1] / 255.;
                double b = pixel[2] / 255.;
                
                row_cumulative_sum += Eigen::Vector3d(r, g, b);
                
                sumimage[image.width * row + col] = row_cumulative_sum;
            }
        }
    }
};

#endif
