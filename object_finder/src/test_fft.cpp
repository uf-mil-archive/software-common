#include <chrono>
#include <iostream>

#include "fft.h"

int main() {
  Eigen::ArrayXXd image = 0.001*Eigen::ArrayXXd::Random(480, 640);
  image.block(5, 5, 3, 3) << 2, 4, 6, 8, 10, 12, 14, 16, 18;
  
  Eigen::ArrayXXd template_ = (Eigen::ArrayXXd(3, 3) <<
    1, 2, 3,
    4, 5, 6,
    7, 8, 9).finished();
  Eigen::ArrayXXd template_weight = Eigen::ArrayXXd::Ones(3, 3);
  
  Eigen::ArrayXXd res;
  
  for(unsigned int i = 0; i < 10; i++) {
    object_finder::fft::calc_pcc(image, template_, template_weight, 0, 0, image.rows()-template_.rows()+1, image.cols()-template_.cols()+1, &res);
  }
  
  auto t1 = std::chrono::high_resolution_clock::now();
  for(unsigned int i = 0; i < 100; i++) {
    object_finder::fft::calc_pcc(image, template_, template_weight, 0, 0, image.rows()-template_.rows()+1, image.cols()-template_.cols()+1, &res);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "calc_pcc took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()/100.
            << " milliseconds\n";
  
  //std::cout << res;
}
