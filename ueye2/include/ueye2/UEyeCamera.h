#ifndef UEYECAMERA_H
#define UEYECAMERA_H

#include <uEye.h>

#include <stdexcept>
#include <stdint.h>
#include <cstdlib>
#include <vector>

class UEyeException : public std::runtime_error {
public:
    UEyeException(const std::string &what) : std::runtime_error(what) { }
};

class UEyeCamera {
public:
    UEyeCamera(unsigned int id,
               unsigned int hbins, unsigned int vbins,
               unsigned int buffered_images);
    ~UEyeCamera();

    unsigned int getWidth() const { return info.nMaxWidth / hbins; }
    unsigned int getHeight() const { return info.nMaxHeight / vbins; }
    
    void start();
    void stop();
    bool isStarted() const { return started; }

    enum AutoFunction {
        AUTO_WHITEBALANCE,
        AUTO_SHUTTER,
        AUTO_GAIN,
        AUTO_FRAMERATE,
    };
    void setAutoFunction(AutoFunction func, bool enabled);
    void setAutoBrightReference(double ref);
    
    void setGains(double r, double g, double b);
    void setFrameRate(double fps);
    
    bool getBayeredImage(uint8_t *buf, size_t len, unsigned int timeout_ms);
    
private:
    unsigned int hbins;
    unsigned int vbins;
    bool opened;
    bool started;

    HIDS cam;
    SENSORINFO info;

    struct ImageMem {
        char *mem;
        INT id;
    };
    std::vector<ImageMem> imagemems;

    void setAutoParameter(INT setting, double val);
    void setAutoParameter(INT setting, double val1, double val2);
};

#endif
