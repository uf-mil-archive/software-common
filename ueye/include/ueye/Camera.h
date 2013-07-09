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

#ifndef UEYE_CAMERA_H_
#define UEYE_CAMERA_H_

#include <uEye.h>
#include <opencv/cv.h>
#include <stdexcept>
#include <string>
#include <boost/function.hpp>
#include <boost/thread.hpp>

using namespace std;

namespace ueye{

	struct uEyeException : public std::runtime_error
	{
		int error_code;
		uEyeException(int code, const char* msg)
			: std::runtime_error(msg), error_code(code)
		{}
	};
	
	enum AWBMode{
	    AWB_DISABLE = IS_AUTOPARAMETER_DISABLE,
	    AWB_AUTO = IS_AUTOPARAMETER_ENABLE,
	    AWB_ONE_SHOT = IS_AUTOPARAMETER_ENABLE_RUNONCE,
    };
	enum uEyeColorSpace{
		SRGB_D50	    = RGB_COLOR_MODEL_SRGB_D50,
		SRGB_D65	    = RGB_COLOR_MODEL_SRGB_D65,
		CIE_RGB_E	    = RGB_COLOR_MODEL_CIE_RGB_E,
		ECI_RGB_D50	    = RGB_COLOR_MODEL_ECI_RGB_D50,
		ADOBE_RGB_D65	= RGB_COLOR_MODEL_ADOBE_RGB_D65,
	};
	enum uEyeColor{
		RGB		= IS_CM_BGR8_PACKED,
		YCbYCr	= IS_CM_CBYCRY_PACKED,
	};
	enum TriggerMode{
		TRIGGER_OFF			= IS_SET_TRIGGER_OFF,
		TRIGGER_HI_LO		= IS_SET_TRIGGER_HI_LO,
		TRIGGER_LO_HI		= IS_SET_TRIGGER_LO_HI,
		TRIGGER_SOFTWARE	= IS_SET_TRIGGER_SOFTWARE,
		TRIGGER_HI_LO_SYNC	= IS_SET_TRIGGER_HI_LO_SYNC,
		TRIGGER_LO_HI_SYNC	= IS_SET_TRIGGER_LO_HI_SYNC,
	};
	enum FlashMode{
		FLASH_OFF				= IO_FLASH_MODE_OFF,
		FLASH_TRIGGER_ACTIVE_LO	= IO_FLASH_MODE_TRIGGER_LO_ACTIVE,
		FLASH_TRIGGER_ACTIVE_HI	= IO_FLASH_MODE_TRIGGER_HI_ACTIVE,
		FLASH_CONSTANT_HIGH		= IO_FLASH_MODE_CONSTANT_HIGH,
		FLASH_CONSTANT_LOW		= IO_FLASH_MODE_CONSTANT_LOW,
		FLASH_FREERUN_ACTIVE_LO	= IO_FLASH_MODE_FREERUN_LO_ACTIVE,
		FLASH_FREERUN_ACTIVE_HI = IO_FLASH_MODE_FREERUN_HI_ACTIVE,
	};

	class Camera
	{
	public:
		Camera();
		~Camera();

		// Initialization functions in order they should be called.
		bool checkVersion(int &Major, int &Minor, int &Build, char *&Expected);
		int getNumberOfCameras();
		unsigned int getSerialNumberList(vector<unsigned int>& SerNo, vector<unsigned int>& DevId);
		bool openCameraCamId(unsigned int id);
		bool openCameraDevId(unsigned int id);
		bool openCameraSerNo(unsigned int serial_number);

		// Get Properties
		char * getCameraName();
		unsigned int getCameraSerialNo();
		int getWidthMax();
		int getHeightMax();
		int getWidth();
		int getHeight();
		int getZoom();
		bool getAutoExposure();
		double getExposure();
		bool getHardwareGamma();
		int getPixelClock();
		bool getGainBoost();
		bool getAutoGain();
		unsigned int getHardwareGain();
		unsigned int getColorTemperature();
		unsigned int getSupportedColorSpaces();
		TriggerMode getTriggerMode();
		TriggerMode getSupportedTriggers();

		// Set Properties
		bool setColorSpace(uEyeColorSpace colspc);
		void setColorTemperature(unsigned int *color_temp);
		void setColorMode(uEyeColor mode);
		void setAWBMode(AWBMode mode);
		void setAutoExposure(bool *Enable);
		void setExposure(double *time_ms);
		void setHardwareGamma(bool *Enable);
		void setZoom(int *zoom);
		void setPixelClock(int *MHz);
		void setFrameRate(double *rate);
		void setGainBoost(bool *Enable);
		void setAutoGain(bool *Enable);
		void setHardwareGain(int *gain);
		bool setTriggerMode(TriggerMode mode);
		void setFlashWithGlobalParams(FlashMode mode);

		bool forceTrigger();

		typedef boost::function<void (IplImage *)> camCaptureCB;
		void startVideoCapture(camCaptureCB);
		void stopVideoCapture();

		void closeCamera();

	private:
		void CheckError(INT err);
		void InitPrivateVariables();
		int getSubsampleParam(int *scale);
		int getBinningParam(int *scale);
		void flashUpdateGlobalParams();

		char **imgMem_;
		int  *imgMemId_;
		int NumBuffers_;
		void initMemoryPool(int size);
		void destroyMemoryPool();
		void captureThread(camCaptureCB captureCallback);
		void restartVideoCapture();

		uEyeColor ColorMode_;
		unsigned int ColorTemperature_;
		AWBMode AWBMode_;
		bool AutoExposure_;
		double ExposureTime_;
		bool HardwareGamma_;
		bool GainBoost_;
		int Zoom_;
		int PixelClock_;
		bool AutoGain_;
		int HardwareGain_;
		double FrameRate_;
		bool FlashGlobalParams_;
		HIDS hCam_;
		SENSORINFO camInfo_;
		unsigned int serialNo_;

		bool Streaming_;
		bool StopCapture_;
		camCaptureCB StreamCallback_;
		boost::thread VidThread_;

	};
}//namespace ueye

#endif /* UEYE_CAMERA_H_ */
