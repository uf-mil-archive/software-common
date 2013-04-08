#ifndef IMAGE_H
#define IMAGE_H

#include <vector>

#include <Eigen/Dense>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


struct Result {
    Eigen::Vector3d total_color;
    Eigen::Vector3d total_color2;
    double count;
    
    Result() {}
    Result(Eigen::Vector3d total_color, Eigen::Vector3d total_color2, double count) :
        total_color(total_color), total_color2(total_color2), count(count) { }
    
    static Result Zero() {
        return Result(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0);
    }
    
    Result operator-(const Result &other) const {
        Result res;
        res.total_color = total_color - other.total_color;
        res.total_color2 = total_color - other.total_color2;
        res.count = count - other.count;
        return res;
    }
    Result operator+(const Result &other) const {
        Result res;
        res.total_color = total_color + other.total_color;
        res.total_color2 = total_color + other.total_color2;
        res.count = count + other.count;
        return res;
    }
    Result operator+=(const Result &other) { return (*this = *this + other); }
    
    Eigen::Vector3d avg_color() const {
        return total_color/count;
    }
    Eigen::Vector3d avg_color2() const {
        return total_color2/count;
    }
};

struct TaggedImage {
    sensor_msgs::CameraInfo cam_info;
    Eigen::Affine3d transform;
    Eigen::Matrix<double, 3, 4> proj;
    std::vector<Eigen::Vector3d> sumimage;
    std::vector<Eigen::Vector3d> sumimage2;
    Result total_result;
    
    
    inline TaggedImage(const sensor_msgs::Image &image, const sensor_msgs::CameraInfo &cam_info, const Eigen::Affine3d &transform) :
        cam_info(cam_info), transform(transform) {
        
        proj = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor> >(cam_info.P.data());
        
        int step;
        if(image.encoding == "rgba8") step = 4;
        else if(image.encoding == "rgb8") step = 3;
        else assert(false);
        
        sumimage.resize((image.width+1) * image.height);
        sumimage2.resize((image.width+1) * image.height);
        for(uint32_t row = 0; row < image.height; row++) {
            Eigen::Vector3d row_cumulative_sum(0, 0, 0);
            Eigen::Vector3d row2_cumulative_sum(0, 0, 0);
            sumimage[(image.width+1) * row + 0] = row_cumulative_sum;
            sumimage2[(image.width+1) * row + 0] = row2_cumulative_sum;
            for(uint32_t col = 0; col < image.width; col++) {
                const uint8_t *pixel = image.data.data() + image.step * row + step * col;
                Eigen::Vector3d color = Eigen::Vector3d(pixel[0] / 255., pixel[1] / 255., pixel[2] / 255.);
                
                row_cumulative_sum += color;
                sumimage[(image.width+1) * row + col+1] = row_cumulative_sum;
                
                row2_cumulative_sum += color.cwiseProduct(color);
                sumimage2[(image.width+1) * row + col+1] = row2_cumulative_sum;
            }
        }
        
        total_result = Result::Zero();
        for(uint32_t row = 0; row < image.height; row++) {
            total_result += get_line_sum(row, 0, cam_info.width);
        }
    }
    
    inline Result get_line_sum(int Y, int X1, int X2) const {
        return Result(
            sumimage [Y * (cam_info.width+1) + X2] - sumimage [Y * (cam_info.width+1) + X1],
            sumimage2[Y * (cam_info.width+1) + X2] - sumimage2[Y * (cam_info.width+1) + X1],
            X2 - X1);
    }
};

#endif
