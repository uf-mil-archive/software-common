#include "ueye2/UEyeCamera.h"

#include <string>
#include <sstream>
#include <cstring>
#include <cassert>

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

#include <iostream>

UEyeCamera::UEyeCamera(unsigned int id, unsigned int hbins, unsigned int vbins, unsigned int images) :
    hbins(hbins),
    vbins(vbins),
    started(false),
    cam(id)
{
    // Initialize camera
    CHECK_ERROR(is_InitCamera, &cam, 0);
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
}

UEyeCamera::~UEyeCamera() {
    if (started) {
        is_StopLiveVideo(cam, IS_FORCE_VIDEO_STOP);
    }
    is_ClearSequence(cam);
    for (unsigned int i = 0; i < imagemems.size(); i++) {
        is_FreeImageMem(cam, imagemems[i].mem, imagemems[i].id);
    }
    is_ExitCamera(cam);
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
    
bool UEyeCamera::getBayeredImage(uint8_t *buf, size_t len) {
    char *mem;
    char *mem_last;
    INT id;
    CHECK_ERROR(is_GetActSeqBuf, cam, &id, &mem, &mem_last);

    INT status = is_WaitEvent(cam, IS_SET_EVENT_FRAME, 1000);
    if (status == IS_WAIT_TIMEOUT) {
        return false;
    }
    _check_error(status, "is_WaitEvent");
    
    CHECK_ERROR(is_LockSeqBuf, cam, id, mem);
    memcpy(buf, mem, len);
    CHECK_ERROR(is_UnlockSeqBuf, cam, id, mem);
    return true;
}
