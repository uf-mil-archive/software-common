#ifndef UEYE_CAMERA_H
#define UEYE_CAMERA_H

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
    UEyeCamera(unsigned int id, unsigned int hbins, unsigned int vbins, unsigned int buffered_images);
    ~UEyeCamera();

    unsigned int getWidth() const { return info.nMaxWidth / hbins; }
    unsigned int getHeight() const { return info.nMaxHeight / vbins; }
    
    void start();
    void stop();
    
    bool getBayeredImage(uint8_t *buf, size_t len);

private:
    unsigned int hbins;
    unsigned int vbins;
    bool started;

    HIDS cam;
    SENSORINFO info;

    struct ImageMem {
        char *mem;
        INT id;
    };
    std::vector<ImageMem> imagemems;
};

#endif
