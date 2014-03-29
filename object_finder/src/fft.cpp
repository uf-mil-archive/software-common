#include <stdexcept>
#include <memory>
#include <unordered_map>

#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/utility.hpp>
#include <Eigen/Dense>
#include <fftw3.h>

#include "fft.h"


namespace object_finder {
namespace fft {


template<typename T>
class FFTWMem : boost::noncopyable {
  T* data;
public:
  FFTWMem(int count) {
    data = reinterpret_cast<T*>(fftw_malloc(sizeof(T) * std::max(1, count)));
    if(!data) {
      throw std::bad_alloc();
    }
  }
  operator T*() {
    return data;
  }
  ~FFTWMem() {
    fftw_free(data);
  }
};

class FFTWPlan : boost::noncopyable {
  fftw_plan p;
public:
  FFTWPlan(fftw_plan p) :
    p(p) {
    if(!p) {
      throw std::runtime_error("FFTWPlan initialized with a null fftw_plan");
    }
  }
  void execute() {
    fftw_execute(p);
  }
  ~FFTWPlan() {
    fftw_destroy_plan(p);
  }
};

typedef Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RealArray;
typedef Eigen::Array<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ComplexArray;

class FFT {
  unsigned int height, width;
  FFTWMem<double> real;
  FFTWMem<fftw_complex> complex;
  FFTWPlan complex_from_real;
  FFTWPlan real_from_complex;
public:
  FFT(unsigned int height, unsigned int width) :
    height(height),
    width(width),
    real(height * width),
    complex(height * (width/2 + 1)),
    complex_from_real(fftw_plan_dft_r2c_2d(
      height, width, real, complex, FFTW_MEASURE|FFTW_DESTROY_INPUT)),
    real_from_complex(fftw_plan_dft_c2r_2d(
      height, width, complex, real, FFTW_MEASURE|FFTW_DESTROY_INPUT)) {
  }
  ComplexArray rfft(RealArray data) {
    assert(data.rows() == height);
    assert(data.cols() == width);
    Eigen::Map<RealArray>(real, height, width) = data;
    complex_from_real.execute();
    return Eigen::Map<ComplexArray>(reinterpret_cast<std::complex<double>*>(static_cast<fftw_complex*>(complex)), height, width/2 + 1);
  }
  RealArray irfft(ComplexArray data) {
    assert(data.rows() == height);
    assert(data.cols() == width/2 + 1);
    Eigen::Map<ComplexArray>(reinterpret_cast<std::complex<double>*>(static_cast<fftw_complex*>(complex)), height, width/2 + 1) = data;
    real_from_complex.execute();
    return Eigen::Map<RealArray>(real, height, width)*(1./(height*width));
  }
};
FFT& get_fft(unsigned int height, unsigned int width) {
  typedef std::pair<unsigned int, unsigned int> KeyType;
  struct Hash {
    size_t operator()(KeyType const & key) const {
      return key.first + 57*key.second;
    }
  };
  static std::unordered_map<KeyType, FFT, Hash> ffts;
  KeyType key = std::make_pair(height, width);
  if(!ffts.count(key)) {
    ffts.emplace(std::piecewise_construct,
      std::forward_as_tuple(key),
      std::forward_as_tuple(height, width));
  }
  return ffts.at(key);
}

uint32_t getGoodDFTSize(uint32_t x) {
  static boost::optional<std::vector<uint32_t>> maybeGoodSizes;
  if(!maybeGoodSizes) {
    std::vector<uint32_t> res;
    for(uint64_t a = 1; a <= std::numeric_limits<uint32_t>::max(); a *= 2) {
      for(uint64_t b = a; b <= std::numeric_limits<uint32_t>::max(); b *= 3) {
        for(uint64_t c = b; c <= std::numeric_limits<uint32_t>::max(); c *= 5) {
          res.push_back(c);
        }
      }
    }
    std::sort(res.begin(), res.end());
    maybeGoodSizes = boost::in_place(res.begin(), res.end());
  }
  
  BOOST_FOREACH(uint32_t y, *maybeGoodSizes) {
    if(y >= x) {
      return y;
    }
  }
  
  return x;
}

typedef std::pair<int, int> Interval;

Interval intersect(Interval const & a, Interval const & b) {
  return std::make_pair(
    std::max(a.first,  b.first),
    std::min(a.second, b.second));
}

// calculates Pearson correlation coefficient
void calc_pcc(Eigen::ArrayXXd const & image,
              //Eigen::ArrayXXd const & image_weight,
              Eigen::ArrayXXd const & template_,
              Eigen::ArrayXXd const & template_weight,
              int res_top,
              int res_left,
              unsigned int res_height,
              unsigned int res_width,
              Eigen::ArrayXXd * res) {
  
  assert(template_.rows() == template_weight.rows());
  assert(template_.cols() == template_weight.cols());
  assert(res);
  
  unsigned int fft_height = getGoodDFTSize(template_.rows() + res_height - 1);
  unsigned int fft_width  = getGoodDFTSize(template_.cols() + res_width  - 1);
  
  FFT &fft = get_fft(fft_height, fft_width);
  
  Eigen::ArrayXXd image_padded = Eigen::ArrayXXd::Zero(fft_height, fft_width);
  Interval image_y = intersect(Interval(res_top , res_top  + res_height + template_.rows() - 1), Interval(0, image.rows()));
  Interval image_x = intersect(Interval(res_left, res_left + res_height + template_.cols() - 1), Interval(0, image.cols()));
  if(image_y.second > image_y.first && image_x.second > image_x.first) {
    image_padded.block(image_y.first - res_top, image_x.first - res_left, image_y.second-image_y.first, image_x.second-image_x.first) =
           image.block(image_y.first,           image_x.first,            image_y.second-image_y.first, image_x.second-image_x.first);
  }
  ComplexArray image_padded_fft = fft.rfft(image_padded);
  
  Eigen::ArrayXXd template_normalized = template_ - (template_weight*template_).sum()/template_weight.sum();
  Eigen::ArrayXXd template_normalized_padded = Eigen::ArrayXXd::Zero(fft_height, fft_width);
  template_normalized_padded.topLeftCorner(template_.rows(), template_.cols()) = template_normalized;
  
  Eigen::ArrayXXd template_weight_padded = Eigen::ArrayXXd::Zero(fft_height, fft_width);
  template_weight_padded.topLeftCorner(template_.rows(), template_.cols()) = template_weight*(1/template_weight.sum());
  ComplexArray template_weight_padded_fft = fft.rfft(template_weight_padded);
  
  #define CROSSCORR(a_fft, b_fft) (fft.irfft(fft.rfft((a_fft)) * fft.rfft((b_fft)).conjugate()))
  /* Eigen::ArrayXXd res_padded = (
    CROSSCORR(image_padded, template_weight_padded*template_normalized_padded) -
    (template_weight_padded*template_normalized_padded).sum() * CROSSCORR(image_padded, template_weight_padded)
  ) / (
    (CROSSCORR(image_padded.square(), template_weight_padded) - CROSSCORR(image_padded, template_weight_padded).square()) *
    (template_weight_padded*template_normalized_padded.square()).sum()
  ).sqrt(); */
  
  #define CROSSCORRF(a_fft, b_fft) ((a_fft) * (b_fft).conjugate())
  Eigen::ArrayXXd res_padded = fft.irfft(
    CROSSCORRF(image_padded_fft, fft.rfft(template_weight_padded*template_normalized_padded)) -
    std::complex<double>((template_weight_padded*template_normalized_padded).sum()) * CROSSCORRF(image_padded_fft, template_weight_padded_fft)
  ) / (
    (fft.irfft(CROSSCORRF(fft.rfft(image_padded.square()), template_weight_padded_fft)) - fft.irfft(CROSSCORRF(image_padded_fft, template_weight_padded_fft)).square()) *
    (template_weight_padded*template_normalized_padded.square()).sum()
  ).sqrt();
  
  *res = res_padded.topLeftCorner(res_height, res_width);
}


}
}
