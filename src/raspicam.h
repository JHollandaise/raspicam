/**********************************************************
 Software developed by AVA ( Ava Group of the University of Cordoba, ava  at uco dot es)
 Main author Rafael Munoz Salinas (rmsalinas at uco dot es)
 This software is released under BSD license as expressed below
-------------------------------------------------------------------
Copyright (c) 2013, AVA ( Ava Group University of Cordoba, ava  at uco dot es)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:

   This product includes software developed by the Ava group of the University of Cordoba.

4. Neither the name of the University nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AVA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL AVA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************/

#ifndef _RaspiCam_H_
#define _RaspiCam_H_

#include <cstdio>
#include <string>
#include <mutex>

#include "mmal/mmal_types.h"
#include "mmal/mmal.h"

#include "raspicamtypes.h"

namespace raspicam {

/**Base class that do all the hard work
 */
class RaspiCam {
 public:
  /**Constructor
   */
  RaspiCam();
  /**Destructor
   */
  ~RaspiCam();

  /**Opens the camera
  * @param StartCapture determines if camera must start capture or not.
        */
  bool Open(bool startCapture = true);
  /**Makes camera start capturing
   */
  bool StartCapture();
  /**Grabs the next frame and keeps it in internal buffer. Blocks until next frame arrives
  */
  bool Grab();
  /**Retrieves the buffer previously grabbed.
   * You can decide how image is given by setting type. The input buffer provided must have the appropriate
   * size accordingly. You can use getImageTypeSize() to determine the size
      * NOTE: Change in version 0.0.5. Format is stablished in setFormat function
      * So type param is ignored. Do not use this parameter.
      * You can use getFormat() to know the current format
    */
  void Retrieve(unsigned char *data, RASPICAM_FORMAT type = RASPICAM_FORMAT_IGNORE);
  /**Alternative to retrieve. Returns a pointer to the original image data buffer (which is in getFormat() format).
    *
    * Be careful, if you call grab(), this will be rewritten with the new data
    */
  unsigned char *GetImageBufferData() const;
  /**
   * Returns the size of the images captured.
   */
  size_t GetImageBufferSize() const;

  /** Stops camera and free resources
  */
  void Release();

  /**Sets capture format
   */
  void SetFormat(RASPICAM_FORMAT fmt);
  /**Sets camera width. Use a multiple of 320 (640, 1280)
   */
  void SetWidth(unsigned int width);
  /**Sets camera Height. Use a multiple of 240 (480, 960)
   */
  void SetHeight(unsigned int height);
  void SetCaptureSize(unsigned int width, unsigned int height);
  /** Set image brightness [0,100]
   */
  void SetBrightness(unsigned int brightness);
  /**
   * Set image sharpness (-100 to 100)
   */
  void SetSharpness(int sharpness);
  /**
   *  Set image contrast (-100 to 100)
   */
  void SetContrast(int contrast);
  /**
   * Set capture ISO (100 to 800)
   */
  void SetISO(int iso);
  /**
   * Set image saturation (-100 to 100)
   */
  void SetSaturation(int saturation);
  /**Sets on/off video stabilisation
   */
  void SetVideoStabilization(bool v);
  /**
   *  Set EV compensation (-10,10)
   */
  void SetExposureCompensation(int val); //-10,10
  void SetRotation(int rotation);
  void SetExposure(RASPICAM_EXPOSURE exposure);
  void SetShutterSpeed(unsigned int ss);
  void SetAWB(RASPICAM_AWB awb);
  // set specific values for white-balance. Requires to set seAWB in OFF mode first
  void SetAWB_RB(float r, float b);//range is 0-1.
  void SetImageEffect(RASPICAM_IMAGE_EFFECT imageEffect);
  void SetMetering(RASPICAM_METERING metering);
  void SetHorizontalFlip(bool hFlip);
  void SetVerticalFlip(bool vFlip);
  void SetFrameRate(unsigned int fr);
  // provides on-line frame rate adjustment
  void SetFramerateDelta(MMAL_RATIONAL_T setpoint);
  //Accessors
  RASPICAM_FORMAT GetFormat() const {return state.capture_fmt;}
  unsigned int GetWidth() const {return state.width;}
  unsigned int GetHeight() const {return state.height;}
  unsigned int GetFrameRate() const {return state.framerate;}
  void GetFramerateDelta(MMAL_RATIONAL_T *value) const;
  unsigned int GetBrightness() const {return state.brightness;}
  unsigned int GetRotation() const {return state.rotation;}
  unsigned int GetISO() const {return state.iso;}
  int GetSharpness() const {return state.sharpness;}
  int GetContrast() const  {return state.contrast;}
  int GetSaturation() const  {return state.saturation;}
  unsigned int GetShutterSpeed() const {return state.shutter_speed;}
  RASPICAM_EXPOSURE GetExposure() const {return state.rpc_exposure_mode;}
  RASPICAM_AWB GetAWB() const {return state.rpc_awb_mode;}
  float GetAWBG_red() const {return state.awbg_red;}
  float GetAWBG_blue() const {return state.awbg_blue;}
  RASPICAM_IMAGE_EFFECT GetImageEffect() const {return state.rpc_image_effect;}
  RASPICAM_METERING GetMetering() const {return state.rpc_exposure_meter_mode;}
  bool IsHorizontallyFlipped() const {return state.h_flip;}
  bool IsVerticallyFlipped() const {return state.v_flip;}
  // the current delta of framerate will be shovelled into value


  /** Returns an id of the camera. We assume the camera id is the one of the raspberry
  *the id is obtained using raspberry serial number obtained in /proc/cpuinfo
  */
  std::string GetId() const;

  /**Returns the size of the required buffer for the different image types in retrieve
   */
  size_t GetImageTypeSize(RASPICAM_FORMAT type) const;
  //Color conversion
  void ConvertBGR2RGB(unsigned char *in_bgr, unsigned char *out_rgb, int size);

 private:
  /**
   * Set the specified camera to all the specified settings
   * @param camera Pointer to camera component
   * @param params Pointer to parameter block containing parameters
   * @return 0 if successful, none-zero if unsuccessful.
   */
  void CommitParameters();
  void CommitBrightness() const;
  void CommitRotation() const;
  void CommitISO() const;
  void CommitSharpness() const;
  void CommitContrast() const;
  void CommitSaturation() const;
  void CommitShutterSpeed() const;
  void CommitExposure() const;
  void CommitExposureCompensation() const;
  void CommitAWB() const;
  void CommitAWB_RB() const;
  void CommitImageEffect() const;
  void CommitMetering() const;
  void CommitFlips() const;
  void CommitVideoStabilization() const;

  void SetDefaultStateParams();
  MMAL_COMPONENT_T *CreateCameraComponent(RASPIVID_STATE *state);
  void DestroyCameraComponent(RASPIVID_STATE *state);
  static void VideoBufferCallback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

  int ConvertFormat(RASPICAM_FORMAT fmt) const;
  MMAL_PARAM_EXPOSUREMODE_T ConvertExposure(RASPICAM_EXPOSURE exposure) const;
  MMAL_PARAM_AWBMODE_T ConvertAWB(RASPICAM_AWB awb) const;
  MMAL_PARAM_IMAGEFX_T ConvertImageEffect(RASPICAM_IMAGE_EFFECT imageEffect) const;
  MMAL_PARAM_EXPOSUREMETERINGMODE_T ConvertMetering(RASPICAM_METERING metering) const;

  RASPIVID_STATE state;
  MMAL_STATUS_T status;
  MMAL_PORT_T *camera_video_port;
  PORT_USERDATA callback_data;
  bool is_open;
  bool is_capturing;
  MMAL_ES_FORMAT_T *format = nullptr;
};
};
#endif

