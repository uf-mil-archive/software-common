#include "ueye2/UEyeCamera.h"

#include <string>
#include <sstream>
#include <cstring>
#include <cassert>
#include <cmath>

#define CHECK_ERROR(func, ...) _check_error(func(__VA_ARGS__), #func)

static void _check_error(IDSEXP val, const char *name) {
    if (val != IS_SUCCESS) {
        std::stringstream buf;
        buf << name << " failed with code " << val;
        throw UEyeException(buf.str());
    }
}

static const INT horiz_masks[] = {
    IS_BINNING_DISABLE,
    IS_BINNING_2X_HORIZONTAL,
    IS_BINNING_3X_HORIZONTAL,
    IS_BINNING_4X_HORIZONTAL,
    IS_BINNING_5X_HORIZONTAL,
    IS_BINNING_6X_HORIZONTAL
};

static const INT vert_masks[] = {
    IS_BINNING_DISABLE,
    IS_BINNING_2X_VERTICAL,
    IS_BINNING_3X_VERTICAL,
    IS_BINNING_4X_VERTICAL,
    IS_BINNING_5X_VERTICAL,
    IS_BINNING_6X_VERTICAL
};

UEyeCamera::UEyeCamera(unsigned int id,
                       unsigned int hbins, unsigned int vbins,
                       unsigned int images) :
    hbins(hbins),
    vbins(vbins),
    opened(false),
    started(false),
    cam(id)
{
    // Initialize camera
    CHECK_ERROR(is_InitCamera, &cam, 0);
    opened = true;
    CHECK_ERROR(is_GetSensorInfo, cam, &info);

    // Allocate memory for images, add to ring buffer
    for (unsigned int i = 0; i < images; i++) {
        ImageMem imagemem;
        CHECK_ERROR(is_AllocImageMem, cam,
                    getWidth(), getHeight(), 32,
                    &imagemem.mem, &imagemem.id);
        CHECK_ERROR(is_AddToSequence, cam, imagemem.mem, imagemem.id);
        imagemems.push_back(imagemem);
    }

    // Configure camera
    assert(0 < hbins && hbins < 7 && 0 < vbins && vbins < 7);
    CHECK_ERROR(is_SetBinning, cam, horiz_masks[hbins-1] | vert_masks[vbins-1]);
    CHECK_ERROR(is_SetColorMode, cam, IS_CM_SENSOR_RAW8);
    setAutoParameter(IS_SET_AUTO_WB_GAIN_RANGE, IS_MIN_AUTO_WB_OFFSET, IS_MAX_AUTO_WB_OFFSET);
}

UEyeCamera::~UEyeCamera() {
    if (opened) {
        if (started) {
            is_StopLiveVideo(cam, IS_FORCE_VIDEO_STOP);
        }
        is_ClearSequence(cam);
        for (unsigned int i = 0; i < imagemems.size(); i++) {
            is_FreeImageMem(cam, imagemems[i].mem, imagemems[i].id);
        }
        is_ExitCamera(cam);
    }
}

void UEyeCamera::start() {
    if (started) {
        return;
    }
    CHECK_ERROR(is_EnableEvent, cam, IS_SET_EVENT_FRAME);
    CHECK_ERROR(is_CaptureVideo, cam, IS_DONT_WAIT);
    started = true;
}

void UEyeCamera::stop() {
    if (!started) {
        return;
    }
    CHECK_ERROR(is_StopLiveVideo, cam, IS_FORCE_VIDEO_STOP);
    started = false;
}

void UEyeCamera::setAutoFunction(AutoFunction func, bool enabled) {
    if (func == AUTO_WHITEBALANCE) {
        // is_AutoParameter is replacing setAutoParameter, but atm it looks like
        // it only works for white balance.
        UINT enable = enabled ? IS_AUTOPARAMETER_ENABLE : IS_AUTOPARAMETER_DISABLE;
        is_AutoParameter(cam, IS_AWB_CMD_SET_ENABLE,
                         reinterpret_cast<void *>(&enable), sizeof(enable));
    } else {
        INT funcs[] = { // indexes corresponds to AutoFunction
            0,
            IS_SET_ENABLE_AUTO_SHUTTER,
            IS_SET_ENABLE_AUTO_GAIN,
            IS_SET_ENABLE_AUTO_FRAMERATE
        };
    
        setAutoParameter(funcs[func], enabled ? 1.0 : 0.0);
    }
}

void UEyeCamera::setAutoBrightReference(double ref) {
    setAutoParameter(IS_SET_AUTO_REFERENCE, floor(ref*255.0));
}

void UEyeCamera::setGains(double r, double g, double b) {
    is_SetHWGainFactor(cam, IS_SET_RED_GAIN_FACTOR, static_cast<int>(r*100));
    is_SetHWGainFactor(cam, IS_SET_GREEN_GAIN_FACTOR, static_cast<int>(g*100));
    is_SetHWGainFactor(cam, IS_SET_BLUE_GAIN_FACTOR, static_cast<int>(b*100));
}

void UEyeCamera::setFrameRate(double fps) {
    CHECK_ERROR(is_SetFrameRate, cam, fps, &fps);
}

bool UEyeCamera::getBayeredImage(uint8_t *buf, size_t len, unsigned int timeout_ms) {
    char *mem;
    char *mem_last;
    INT id;
    CHECK_ERROR(is_GetActSeqBuf, cam, &id, &mem, &mem_last);

    INT status = is_WaitEvent(cam, IS_SET_EVENT_FRAME, timeout_ms);
    if (status == IS_TIMED_OUT) {
        return false;
    }
    _check_error(status, "is_WaitEvent");
    
    CHECK_ERROR(is_LockSeqBuf, cam, id, mem);
    memcpy(buf, mem, len);
    CHECK_ERROR(is_UnlockSeqBuf, cam, id, mem);
    return true;
}

void UEyeCamera::setAutoParameter(INT setting, double val) {
    CHECK_ERROR(is_SetAutoParameter, cam,
                setting, &val, NULL);
}

void UEyeCamera::setAutoParameter(INT setting, double val1, double val2) {
    CHECK_ERROR(is_SetAutoParameter, cam,
                setting, &val1, &val2);
}
