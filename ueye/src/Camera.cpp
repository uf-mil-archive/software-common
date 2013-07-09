/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Kevin Hallenbeck
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Kevin Hallenbeck nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "ueye/Camera.h"

using namespace std;

#define DEBUG_ERROR_CHECKS 0

#if DEBUG_ERROR_CHECKS
#define CHECK_ERR	CheckError
#else
#define CHECK_ERR(fnc)	                        			\
do {                                                       	\
	IS_CHAR* msg;											\
	int err = fnc;                                       	\
	if (err != IS_SUCCESS) {								\
		if (hCam_ != 0){									\
			is_GetError(hCam_, &err, &msg);					\
			if(err != IS_SUCCESS){							\
				throw uEyeException(err, msg);				\
			}												\
		}else{												\
			throw uEyeException(err, "Camera failed to initialize");\
		}													\
	}                                                       \
} while (false)
#endif

namespace ueye{

void Camera::CheckError(INT err)
{
	INT err2 = IS_SUCCESS;
	IS_CHAR* msg;
	if (err != IS_SUCCESS) {
		if (hCam_ != 0){
			is_GetError(hCam_, &err2, &msg);
			if(err2 != IS_SUCCESS){
				throw ueye::uEyeException(err, msg);
			}
		}else{
			throw ueye::uEyeException(err, "Camera failed to initialize");
		}
	}
}

void Camera::InitPrivateVariables()
{
	Streaming_ = false;
	StopCapture_ = false;
	ColorMode_ = RGB;
	AWBMode_ = AWB_DISABLE;
	AutoExposure_ = false;
	ExposureTime_ = 99.0;
	HardwareGamma_ = true;
	GainBoost_ = false;
	Zoom_ = 1;
	PixelClock_ = 35;
	AutoGain_ = false;
	HardwareGain_ = 100;
	FrameRate_ = 5.0;
	FlashGlobalParams_ = false;
	serialNo_ = 0;
	hCam_ = 0;
	memset(&camInfo_, 0x00, sizeof(camInfo_));
	NumBuffers_ = 0;
	StreamCallback_ = NULL;
	ColorTemperature_ = 5000;
}

Camera::Camera()
{
	InitPrivateVariables();
}

bool Camera::checkVersion(int &Major, int &Minor, int &Build, char *&Expected)
{
	Expected = (char*)"4.20.6";
	Build = is_GetDLLVersion();
	Major = (Build >> 24) & 0x000000FF;
	Minor = (Build >> 16) & 0x000000FF;
	Build &= 0x0000FFFF;
	if((Major == 4) && (Minor == 20) && (Build == 6)){
		return true;
	}
	return false;
}

int Camera::getNumberOfCameras()
{
	int Num = 0;
	CHECK_ERR(is_GetNumberOfCameras(&Num));
	return Num;
}


unsigned int Camera::getSerialNumberList(vector<unsigned int>& SerNo, vector<unsigned int>& DevId)
{
	int num = getNumberOfCameras();
	if( num > 0 ) {
		UEYE_CAMERA_LIST *list = (UEYE_CAMERA_LIST *)malloc(sizeof(DWORD) + num*sizeof(UEYE_CAMERA_INFO));
		list->dwCount = num;
		if (is_GetCameraList(list) == IS_SUCCESS){
			num = list->dwCount;
			SerNo.resize(num);
			DevId.resize(num);
			for(int i=0; i<num; i++){
				SerNo[i] = atoi(list->uci[i].SerNo);
				DevId[i] = list->uci[i].dwDeviceID;
			}
		}else{
			num = 0;
		}
		free(list);
		return num;
	}
	return 0;
}

bool Camera::openCameraCamId(unsigned int id)
{
	if(getNumberOfCameras() < 1){
		return false;
	}

	hCam_ = id;
	CHECK_ERR(is_InitCamera(&hCam_, 0));

	CHECK_ERR(is_GetSensorInfo(hCam_, &camInfo_));
	CAMINFO info;
	CHECK_ERR(is_GetCameraInfo(hCam_, &info));
	serialNo_ = atoi(info.SerNo);

	setColorMode(ColorMode_);
	setAutoExposure(&AutoExposure_);
	if(!AutoExposure_){
		setExposure(&ExposureTime_);
	}
	setHardwareGamma(&HardwareGamma_);
	setGainBoost(&GainBoost_);
	setAutoGain(&AutoGain_);
	if(!AutoGain_){
		setHardwareGain(&HardwareGain_);
	}
	setZoom(&Zoom_);
	setPixelClock(&PixelClock_);
	setFrameRate(&FrameRate_);
	return true;
}
bool Camera::openCameraDevId(unsigned int id)
{
	return openCameraCamId(id | IS_USE_DEVICE_ID);
}
bool Camera::openCameraSerNo(unsigned int serial_number)
{
	vector<unsigned int> SerNo;
	vector<unsigned int> DevId;
	unsigned int num = getSerialNumberList(SerNo, DevId);
	for(unsigned int i=0; i<num; i++){
		if(SerNo[i] == serial_number){
			return openCameraDevId(DevId[i]);
		}
	}
	return false;
}

char * Camera::getCameraName()
{
	return camInfo_.strSensorName;
}
unsigned int Camera::getCameraSerialNo(){
	return serialNo_;
}
int Camera::getZoom()
{
	return Zoom_;
}
int Camera::getWidthMax()
{
	return camInfo_.nMaxWidth;
}
int Camera::getHeightMax()
{
	return camInfo_.nMaxHeight;
}
int Camera::getWidth()
{
	return camInfo_.nMaxWidth / Zoom_;
}
int Camera::getHeight()
{
	return camInfo_.nMaxHeight / Zoom_;
}
bool Camera::getAutoExposure()
{
	return AutoExposure_;
}
double Camera::getExposure()
{
	double time_ms;
	CHECK_ERR(is_Exposure (hCam_, IS_EXPOSURE_CMD_GET_EXPOSURE, &time_ms, sizeof(double)));
	return time_ms;
}
bool Camera::getHardwareGamma()
{
	return HardwareGamma_;
}
int Camera::getPixelClock()
{
	return PixelClock_;
}
unsigned int Camera::getColorTemperature()
{
	return ColorTemperature_;
}
bool Camera::getGainBoost()
{
	return GainBoost_;
}
bool Camera::getAutoGain()
{
	return AutoGain_;
}
unsigned int Camera::getHardwareGain()
{
	HardwareGain_ = is_SetHWGainFactor(hCam_, IS_GET_MASTER_GAIN_FACTOR, 0);
	return HardwareGain_;
}
TriggerMode Camera::getTriggerMode()
{
	return (TriggerMode)is_SetExternalTrigger(hCam_, IS_GET_EXTERNALTRIGGER);
}
TriggerMode Camera::getSupportedTriggers()
{
	return (TriggerMode)is_SetExternalTrigger(hCam_, IS_GET_SUPPORTED_TRIGGER_MODE);
}
unsigned int Camera::getSupportedColorSpaces()
{
    unsigned int bitmask;
    if(IS_SUCCESS == is_ColorTemperature(hCam_,COLOR_TEMPERATURE_CMD_GET_SUPPORTED_RGB_COLOR_MODELS,
                        (void*)&bitmask, sizeof(bitmask)))
        return bitmask;
    else
        return 0;
}
bool Camera::setColorSpace(uEyeColorSpace colspc)
{
	if(colspc & getSupportedTriggers()){
		if(is_ColorTemperature(hCam_, COLOR_TEMPERATURE_CMD_SET_RGB_COLOR_MODEL,
		        (void*)&colspc, sizeof(colspc)) == IS_SUCCESS){
			return true;
		}
	}
	return false;
}
void Camera::setColorMode(uEyeColor mode)
{
	CHECK_ERR(is_SetColorMode(hCam_, mode));
	ColorMode_ = mode;
}
void Camera::setAutoExposure(bool *Enable)
{
	double param1 = *Enable ? 1.0 : 0.0;
	double param2 = 0;
	if(IS_SUCCESS != is_SetAutoParameter(hCam_, IS_SET_ENABLE_AUTO_SHUTTER, &param1, &param2)){
		param1 = 0;
		is_SetAutoParameter(hCam_, IS_SET_ENABLE_AUTO_SHUTTER, &param1, &param2);
		*Enable = false;
	}
	AutoExposure_ = *Enable;
}
void Camera::setAWBMode(AWBMode mode)
{
    CHECK_ERR(is_AutoParameter(hCam_, IS_AWB_CMD_SET_ENABLE, (void*)&mode, sizeof(mode)));
    AWBMode_ = mode;
}
void  Camera::setExposure(double *time_ms)
{
	bool b = false;
	setAutoExposure(&b);
	CHECK_ERR(is_Exposure (hCam_, IS_EXPOSURE_CMD_SET_EXPOSURE, time_ms, sizeof(double)));
	flashUpdateGlobalParams();
	ExposureTime_ = *time_ms;
}
void  Camera::setColorTemperature(unsigned int *color_temp)
{
	AWBMode mode = AWB_DISABLE;
	setAWBMode(mode);
	CHECK_ERR(is_ColorTemperature(hCam_, COLOR_TEMPERATURE_CMD_SET_TEMPERATURE, color_temp, sizeof(int)));
	flashUpdateGlobalParams();
	ColorTemperature_ = *color_temp;
}
void Camera::setHardwareGamma(bool *Enable)
{
	if(*Enable){
		if(IS_SUCCESS != is_SetHardwareGamma(hCam_, IS_SET_HW_GAMMA_ON)){
			is_SetHardwareGamma(hCam_, IS_SET_HW_GAMMA_OFF);
			*Enable = false;
		}
	}else{
		is_SetHardwareGamma(hCam_, IS_SET_HW_GAMMA_OFF);
	}
	HardwareGamma_ = *Enable;
}
void Camera::setZoom(int *zoom)
{
	if(Zoom_ != *zoom){
		// Reset zoom
		is_SetSubSampling(hCam_, 0);
		is_SetBinning(hCam_, 0);

		// Try Subsampling then Binning
		if(IS_SUCCESS != is_SetSubSampling(hCam_, getSubsampleParam(zoom))){
			is_SetSubSampling(hCam_, 0);
			if(IS_SUCCESS != is_SetBinning(hCam_, getBinningParam(zoom))){
				is_SetBinning(hCam_, 0);
				*zoom = 1;
			}
		}

		// Zoom affects the frame-rate and needs a restart to change the buffer sizes
		is_HotPixel(hCam_, IS_HOTPIXEL_ENABLE_CAMERA_CORRECTION, NULL, 0);
		setFrameRate(&FrameRate_);
		restartVideoCapture();
	}
	Zoom_ = *zoom;
}
void Camera::setPixelClock(int *MHz)
{
	int Ranges[3];
	memset(Ranges, 0x00, sizeof(Ranges));

	// Sanitize to increment, minimum, and maximum
	CHECK_ERR(is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_GET_RANGE, Ranges, sizeof(Ranges)));
	if(Ranges[2] > 1){
		if((*MHz - Ranges[0]) % Ranges[2] != 0){
			*MHz -= (*MHz - Ranges[0]) % Ranges[2];
		}
	}
	if(*MHz < Ranges[0]){
		*MHz = Ranges[0];
	}
	if(*MHz > Ranges[1]){
		*MHz = Ranges[1];
	}

	CHECK_ERR(is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_SET, MHz, sizeof(int)));
	setFrameRate(&FrameRate_);
}
void Camera::setFrameRate(double *rate)
{
	CHECK_ERR(is_SetFrameRate(hCam_, *rate, rate));
	flashUpdateGlobalParams();
	FrameRate_ = *rate;
}
void Camera::setGainBoost(bool *enable)
{
	if(is_SetGainBoost(hCam_, IS_GET_SUPPORTED_GAINBOOST) == IS_SET_GAINBOOST_ON){
		if(*enable)
			is_SetGainBoost(hCam_, IS_SET_GAINBOOST_ON);
		else
			is_SetGainBoost(hCam_, IS_SET_GAINBOOST_OFF);
		GainBoost_ = is_SetGainBoost(hCam_, IS_GET_GAINBOOST) == IS_SET_GAINBOOST_ON;
	}else{
		GainBoost_ = false;
	}
	*enable = GainBoost_;
}
void Camera::setAutoGain(bool *Enable)
{
	double param1 = *Enable ? 1.0 : 0.0;
	double param2 = 0;
	if(IS_SUCCESS != is_SetAutoParameter(hCam_, IS_SET_ENABLE_AUTO_GAIN, &param1, &param2)){
		param1 = 0;
		is_SetAutoParameter(hCam_, IS_SET_ENABLE_AUTO_GAIN, &param1, &param2);
		*Enable = false;
	}
	AutoGain_ = *Enable;
}
void Camera::setHardwareGain(int *gain)
{
	bool b = false;
	setAutoGain(&b);
	if(*gain < 0)
		*gain = 0;
	if(*gain > 400)
		*gain = 400;
	HardwareGain_ = is_SetHWGainFactor(hCam_, IS_SET_MASTER_GAIN_FACTOR, *gain);
	*gain = HardwareGain_;
}
bool Camera::setTriggerMode(TriggerMode mode)
{
	if((mode == 0) || (mode & getSupportedTriggers())){
		if(is_SetExternalTrigger(hCam_, mode) == IS_SUCCESS){
			return true;
		}
	}
	return false;
}
void Camera::setFlashWithGlobalParams(FlashMode mode)
{
	UINT nMode = mode;
	switch(mode)
	{
	case FLASH_FREERUN_ACTIVE_LO:
	case FLASH_FREERUN_ACTIVE_HI:
	case FLASH_TRIGGER_ACTIVE_LO:
	case FLASH_TRIGGER_ACTIVE_HI:
		FlashGlobalParams_ = true;
		break;

	case FLASH_CONSTANT_HIGH:
	case FLASH_CONSTANT_LOW:
		FlashGlobalParams_ = false;
		break;

	case FLASH_OFF:
	default:
		FlashGlobalParams_ = false;
		nMode = FLASH_OFF;
		break;
	}
	CHECK_ERR(is_IO(hCam_, IS_IO_CMD_FLASH_SET_MODE, (void*)&nMode, sizeof(nMode)));
	flashUpdateGlobalParams();
}
void Camera::flashUpdateGlobalParams()
{
	if(FlashGlobalParams_){
		IO_FLASH_PARAMS params;
		CHECK_ERR(is_IO(hCam_, IS_IO_CMD_FLASH_GET_GLOBAL_PARAMS,
				(void*)&params, sizeof(params)));
		CHECK_ERR(is_IO(hCam_, IS_IO_CMD_FLASH_APPLY_GLOBAL_PARAMS, NULL, 0));
	}
}

bool Camera::forceTrigger()
{
	if(Streaming_)
		return is_ForceTrigger(hCam_) == IS_SUCCESS;
	return false;
}

int Camera::getSubsampleParam(int *scale)
{
	int param;
	if(*scale == 3){
		*scale = 2;
	}
	switch(*scale){
		case 2:
			param = IS_SUBSAMPLING_2X_VERTICAL | IS_SUBSAMPLING_2X_HORIZONTAL;
			break;
		case 4:
			param = IS_SUBSAMPLING_4X_VERTICAL | IS_SUBSAMPLING_4X_HORIZONTAL;
			break;
		default:
			*scale = 1;
			param = IS_SUBSAMPLING_DISABLE;
			break;
	}
	return param;
}
int Camera::getBinningParam(int *scale)
{
	int param;
	if(*scale == 3){
		*scale = 2;
	}
	switch(*scale){
		case 2:
			param = IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL;
			break;
		case 4:
			param = IS_BINNING_4X_VERTICAL | IS_BINNING_4X_HORIZONTAL;
			break;
		default:
			*scale = 1;
			param = IS_BINNING_DISABLE;
			break;
	}
	return param;
}

void Camera::closeCamera(){
	if(hCam_ > 0){
		// Release camera and all associated memory
		CHECK_ERR(IS_SUCCESS != is_ExitCamera(hCam_));
		InitPrivateVariables();
	}
}

Camera::~Camera(){
	closeCamera();
}

void Camera::initMemoryPool(int size)
{
	int Width = getWidth();
	int Height = getHeight();
	if(size < 2){
		size = 2;
	}
	imgMem_ = new char* [size];
	imgMemId_ = new int [size];
	for (int i = 0; i < size; i++){
		if (IS_SUCCESS != is_AllocImageMem (hCam_, Width, Height, 24, &imgMem_[i], &imgMemId_[i])){
			throw uEyeException(-1, "Failed to initialize memory.");
		}
		//add memory to memory pool
		if (IS_SUCCESS != is_SetImageMem(hCam_, imgMem_[i], imgMemId_[i])){
			throw uEyeException(-1, "Failed to initialize memory.");
		}
	}
	NumBuffers_ = size;
}
void Camera::destroyMemoryPool()
{
	for (int i=0; i<NumBuffers_; i++){
		CHECK_ERR(is_FreeImageMem(hCam_,  imgMem_[i], imgMemId_[i]));
	}
	NumBuffers_ = 0;
}

void Camera::captureThread(camCaptureCB callback)
{
	Streaming_ = true;
	void * imgMem;
	StopCapture_ = false;

	initMemoryPool(4);

	// Setup video event
	CHECK_ERR(is_EnableEvent(hCam_, IS_SET_EVENT_FRAME));
	CHECK_ERR(is_CaptureVideo(hCam_, IS_WAIT));
	IplImage * p_img = cvCreateImageHeader(cvSize(getWidth(),getHeight()), IPL_DEPTH_8U, 3);

	while(!StopCapture_){
		// Wait for image. Timeout after 2*FramePeriod = (2000ms/FrameRate)
		if(is_WaitEvent(hCam_, IS_SET_EVENT_FRAME, (int)(2000/FrameRate_)) == IS_SUCCESS){
			if(is_GetImageMem(hCam_, &imgMem) == IS_SUCCESS){
				p_img->imageData = (char*)imgMem;
				callback(p_img);
			}
		}
	}

	// Stop video event
	CHECK_ERR(is_DisableEvent(hCam_, IS_SET_EVENT_FRAME));
	CHECK_ERR(is_StopLiveVideo(hCam_, IS_WAIT));

	destroyMemoryPool();
	Streaming_ = false;
}

void Camera::startVideoCapture(camCaptureCB callback){
	StreamCallback_ = callback;
	VidThread_ = boost::thread(&Camera::captureThread, this, callback);
}
void Camera::stopVideoCapture(){
	StopCapture_ = true;
	if (VidThread_.joinable()){
		forceTrigger();
		VidThread_.join();
	}
}
void Camera::restartVideoCapture(){
	if(Streaming_){
		if(StreamCallback_ != NULL){
			stopVideoCapture();
			startVideoCapture(StreamCallback_);
		}
	}
}

}//namespace ueye
