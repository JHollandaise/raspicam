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

#include "raspicam.h"

#include <cstdio>
#include <iostream>

#include "mmal/util/mmal_util.h"
#include "mmal/util/mmal_util_params.h"
#include "mmal/util/mmal_default_components.h"

using namespace std;
namespace raspicam {

#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2
#define VIDEO_FRAME_RATE_DEN 1
#define VIDEO_OUTPUT_BUFFERS_NUM 3

RaspiCam::RaspiCam() {
  camera_video_port = nullptr;
  is_open = false;
  is_capturing = false;
  SetDefaultStateParams();
}

RaspiCam::~RaspiCam() {
  Release();
}

bool RaspiCam::Open(bool startCapture) {

  if (is_open) return false;

  // create camera
  if (!CreateCameraComponent(&state)) {
    cerr << __func__ << "Failed to create camera component" << __FILE__ << " " << __LINE__ << endl;
    return false;
  }
  CommitParameters();
  camera_video_port = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
  callback_data.pstate = &state;

  // assign data to use for callback
  // TODO: understand better the flow of pot.userdata <-> this.callback_data
  camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *) &callback_data;

  is_open = true;
  // TODO: bad variable naming
  if (startCapture) return StartCapture();
}

bool RaspiCam::StartCapture() {
  if (!is_open) {
    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << "not opened." << endl;
    return false;
  }

  // start capture
  if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
    Release();
    return false;
  }

  // send all the buffers to the video port
  unsigned int num = mmal_queue_length(state.video_pool->queue);
  int q;
  for (q = 0; q < num; q++) {
    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.video_pool->queue);

    if (!buffer)
      cerr << "Unable to get required buffer " << q << " from pool queue" << endl;

    if (mmal_port_send_buffer(camera_video_port, buffer) != MMAL_SUCCESS)
      cerr << "Unable to send a buffer to encoder output port " << q << endl;
  }
  is_capturing = true;
  return true;
}

void RaspiCam::Release() {
  if (!is_open) return;

  // disable cameraVideoPort
  if (camera_video_port && camera_video_port->is_enabled) {
    mmal_port_disable(camera_video_port);
    camera_video_port = nullptr;
  }
  // disable all our ports that are not handled by connections
  // TODO: this might not be correct, see again
  if (state.camera_component)
    mmal_component_disable(state.camera_component);

  DestroyCameraComponent(&state);

  is_open = false;
  is_capturing = false;
}

bool RaspiCam::Grab() {
  if (!is_capturing) return false;
  callback_data.WaitForFrame();
  return true;
}

void RaspiCam::Retrieve(unsigned char *data, RASPICAM_FORMAT type) {
  if (callback_data._buffData.size == 0) return;
  // TODO: this is you wot m7!?
  if (type != RASPICAM_FORMAT_IGNORE) {
    cerr << __FILE__ << ":" << __LINE__ << " : Retrieve type is not RASPICAM_FORMAT_IGNORE as expected" << endl;
  }
  if (state.capture_fmt == RASPICAM_FORMAT_YUV420) {
    auto imagePtr = callback_data._buffData.data;
    for (int i = 0; i < state.height + state.height / 2; i++) {
      memcpy(data, imagePtr, state.width);
      data += state.width;
      imagePtr += format->es->video.width; // line stride
    }
  } else if (state.capture_fmt == RASPICAM_FORMAT_GRAY) {
    auto imagePtr = callback_data._buffData.data;
    for (int i = 0; i < state.height; i++) {
      memcpy(data, imagePtr, state.width);
      data += state.width;
      imagePtr += format->es->video.width; // line stride
    }
  } else if (state.capture_fmt == RASPICAM_FORMAT_BGR || state.capture_fmt == RASPICAM_FORMAT_RGB) {
    auto imagePtr = callback_data._buffData.data;
    for (int i = 0; i < state.height; i++) {
      memcpy(data, imagePtr, state.width * 3);
      data += state.width * 3;
      imagePtr += format->es->video.width * 3; // line stride
    }
  }
}

unsigned char *RaspiCam::GetImageBufferData() const {
  return callback_data._buffData.data;
}

size_t RaspiCam::GetImageBufferSize() const {
  return GetImageTypeSize(GetFormat());
}

size_t RaspiCam::GetImageTypeSize(RASPICAM_FORMAT type) const {
  switch (type) {
    // TODO: I'm pretty sure this statement is broken
    case RASPICAM_FORMAT_YUV420:return GetWidth()*GetHeight() + 2*((GetWidth()/2*GetHeight()/2));
      break;
    case RASPICAM_FORMAT_GRAY:return GetWidth()*GetHeight();
      break;
    case RASPICAM_FORMAT_BGR:
    case RASPICAM_FORMAT_RGB:return 3*GetWidth()*GetHeight();
      break;
    default:return 0;
  }
}

void RaspiCam::SetFormat(RASPICAM_FORMAT fmt) {
  if (is_open) {
    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
         << ": cannot change format with camera already open" << endl;
    return;
  }
  state.capture_fmt = fmt;
}

void RaspiCam::SetWidth(unsigned int width) {
  state.width = width;
}
void RaspiCam::SetHeight(unsigned int height) {
  state.height = height;
}
void RaspiCam::SetCaptureSize(unsigned int width, unsigned int height) {
  SetWidth(width);
  SetHeight(height);
}

void RaspiCam::SetBrightness(unsigned int brightness) {
  if (brightness > 100) brightness = 100;
  state.brightness = brightness;
  if (is_open) CommitBrightness();
}

void RaspiCam::SetRotation(int rotation) {
  while (rotation < 0) rotation += 360;
  if (rotation >= 360) rotation %= 360;
  state.rotation = rotation;
  if (is_open) CommitRotation();
}

void RaspiCam::SetISO(int iso) {
  state.iso = iso;
  if (is_open) CommitISO();
}

void RaspiCam::SetSharpness(int sharpness) {
  if (sharpness < -100) sharpness = -100;
  if (sharpness > 100) sharpness = 100;
  state.sharpness = sharpness;
  if (is_open) CommitSharpness();
}

void RaspiCam::SetContrast(int contrast) {
  if (contrast < -100) contrast = -100;
  if (contrast > 100) contrast = 100;
  state.contrast = contrast;
  if (is_open) CommitContrast();
}

void RaspiCam::SetSaturation(int saturation) {
  if (saturation < -100) saturation = -100;
  if (saturation > 100) saturation = 100;
  state.saturation = saturation;
  if (is_open) CommitSaturation();
}

void RaspiCam::SetExposure(RASPICAM_EXPOSURE exposure) {
  state.rpc_exposure_mode = exposure;
  if (is_open) CommitExposure();
}

void RaspiCam::SetShutterSpeed(unsigned int ss) {
  // TODO: I don't know what orifice they pulled this max value out of
  if (ss > 330000) ss = 330000;
  state.shutter_speed = ss;
  if (is_open) CommitShutterSpeed();
}

void RaspiCam::SetVideoStabilization(bool v) {
  state.video_stabilisation = v;
  if (is_open) CommitVideoStabilization();
}

void RaspiCam::SetExposureCompensation(int val) {
  if (val < -10) val = -10;
  if (val > 10) val = 10;
  state.exposure_compensation = val;
  if (is_open) CommitExposureCompensation();
}

void RaspiCam::SetAWB(RASPICAM_AWB awb) {
  state.rpc_awb_mode = awb;
  if (is_open) CommitAWB();
}

void RaspiCam::SetAWB_RB(float r, float b) {
  state.awbg_red = r;
  state.awbg_blue = b;
  if (is_open) CommitAWB_RB();
}

void RaspiCam::SetImageEffect(RASPICAM_IMAGE_EFFECT imageEffect) {
  state.rpc_image_effect = imageEffect;
  if (is_open) CommitImageEffect();
}

void RaspiCam::SetMetering(RASPICAM_METERING metering) {
  state.rpc_exposure_meter_mode = metering;
  if (is_open) CommitMetering();
}

void RaspiCam::SetHorizontalFlip(bool hFlip) {
  state.h_flip = hFlip;
  if (is_open) CommitFlips();
}

void RaspiCam::SetVerticalFlip(bool vFlip) {
  state.h_flip = vFlip;
  if (is_open) CommitFlips();
}

void RaspiCam::SetFrameRate(unsigned int fr) {
  state.framerate = fr;
}

// here we are assuming that the framerate is always just an integer value, so check VIDEO_FRAMRE_RATE_DEN is def as 1
#if VIDEO_FRAME_RATE_DEN != 1
#   error "fix framerate setting function to account for the different denominator"
#endif
void RaspiCam::SetFramerateDelta(MMAL_RATIONAL_T setpoint) {
  if (!is_open) {
    cout << __func__ << ": Framerate Delta can only be set when the camera is open" << endl;
  }
  // first get the new framerate required
  setpoint.num += state.framerate * setpoint.den;
  if (mmal_port_parameter_set_rational(state.camera_component->output[MMAL_CAMERA_VIDEO_PORT],
                                       MMAL_PARAMETER_FRAME_RATE, setpoint) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set framerate Delta parameter." << endl;
}


void RaspiCam::GetFramerateDelta(MMAL_RATIONAL_T *value) const {
  // TODO: need to check the camera is currently active?
  // TODO: something about if there is no encoder active use preview port rather than video
  // first read the actual framerate parameter into value
  mmal_port_parameter_get_rational(state.camera_component->output[MMAL_CAMERA_VIDEO_PORT],
                                   MMAL_PARAMETER_FRAME_RATE, value);


  // remove the initialised format framerate from the current value
  value->num -= state.framerate * value->den;
}

//Returns an id of the camera. We assume the camera id is the one of the raspberry
//the id is obtained using raspberry serial number obtained in /proc/cpuinfo
std::string RaspiCam::GetId() const {
  char serial[1024];
  serial[0] = '\0';
  ifstream file ("/proc/cpuinfo");
  if (!file) {
    cerr << __FILE__ << " " << __LINE__ << ":" << __func__ << "Could not read /proc/cpuinfo" << endl;
    return serial;
  }
  //read lines until find serial
  bool found = false;
  while (!file.eof() && !found){
    char line[1024];
    file.getline(line, 1024);
    string str(line);
    char aux[100];

    if (str.find("Serial") != string::npos){
      if (sscanf(line, "%s : %s", aux, serial) !=2) {
        cerr << __FILE__ << " " << __LINE__ << ":" << __func__ << "Error parsing /proc/cpuinfo" << endl;
      } else found = true;
    }
  }
  return serial;
}

void RaspiCam::ConvertBGR2RGB(unsigned char *in_bgr, unsigned char *out_rgb, int size){
  unsigned char *end = in_bgr+size;
  unsigned char *in_ptr = in_bgr;
  while (in_ptr < end) {
    swap (in_ptr[2],in_ptr[0]);
    in_ptr += 3;
  }
  mempcpy(out_rgb, in_bgr, size);

}

void RaspiCam::CommitParameters() {
  assert(state.camera_component != nullptr);
  CommitSaturation();
  CommitSharpness();
  CommitContrast();
  CommitBrightness();
  CommitISO();
  if (state.shutter_speed != 0) {
    CommitShutterSpeed();
    state.rpc_exposure_mode = RASPICAM_EXPOSURE_FIXEDFPS;
  }
  CommitExposure();
  CommitExposureCompensation();
  CommitMetering();
  CommitImageEffect();
  CommitRotation();
  CommitFlips();
  CommitVideoStabilization();
  CommitAWB();
  CommitAWB_RB();
}

void RaspiCam::CommitBrightness() const {
  mmal_port_parameter_set_rational(state.camera_component->control, MMAL_PARAMETER_BRIGHTNESS, {static_cast<int>(state.brightness), 100});
}

void RaspiCam::CommitRotation() const {
  // set to nearest 90 degree
  auto rotation{(state.rotation/90)*90};
  mmal_port_parameter_set_int32(state.camera_component->output[0], MMAL_PARAMETER_ROTATION, rotation);
  mmal_port_parameter_set_int32(state.camera_component->output[1], MMAL_PARAMETER_ROTATION, rotation);
  mmal_port_parameter_set_int32(state.camera_component->output[2], MMAL_PARAMETER_ROTATION, rotation);
}

void RaspiCam::CommitISO() const {
  if(mmal_port_parameter_set_uint32(state.camera_component->control, MMAL_PARAMETER_ISO, state.iso) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set ISO parameter" << endl;
}

void RaspiCam::CommitSharpness() const {
  if (mmal_port_parameter_set_rational(state.camera_component->control, MMAL_PARAMETER_SHARPNESS,
                                       {state.sharpness, 100}) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set sharpness parameter." << endl;
}

void RaspiCam::CommitContrast() const {
  if (mmal_port_parameter_set_rational(state.camera_component->control, MMAL_PARAMETER_CONTRAST,
                                       {state.contrast, 100}) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set contrast parameter." << endl;
}

void RaspiCam::CommitSaturation() const {
  if (mmal_port_parameter_set_rational(state.camera_component->control, MMAL_PARAMETER_SATURATION, {state.saturation, 100}) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set saturation parameter." << endl;
}

void RaspiCam::CommitShutterSpeed() const {
  if (mmal_port_parameter_set_uint32(state.camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, state.shutter_speed) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set shutter parameter." << endl;
}

void RaspiCam::CommitExposure() const {
  MMAL_PARAMETER_EXPOSUREMODE_T exp_mode = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(exp_mode)}, ConvertExposure(state.rpc_exposure_mode)};
  if (mmal_port_parameter_set(state.camera_component->control, &exp_mode.hdr) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set exposure parameter." << endl;
}

void RaspiCam::CommitExposureCompensation() const {
  if (mmal_port_parameter_set_int32(state.camera_component->control, MMAL_PARAMETER_EXPOSURE_COMP, state.exposure_compensation) !=MMAL_SUCCESS)
    cout << __func__ << ": Failed to set Exposure Compensation parameter." << endl;

}

void RaspiCam::CommitAWB() const {
  MMAL_PARAMETER_AWBMODE_T param = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, ConvertAWB(state.rpc_awb_mode)};
  if (mmal_port_parameter_set(state.camera_component->control, &param.hdr) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set AWB parameter." << endl;
}

void RaspiCam::CommitAWB_RB() const {
  MMAL_PARAMETER_AWB_GAINS_T param = {{MMAL_PARAMETER_CUSTOM_AWB_GAINS, sizeof(param)}, {0,0}, {0,0}};
  param.r_gain.num = static_cast<unsigned int>(state.awbg_red * 65536);
  param.b_gain.num = static_cast<unsigned int>(state.awbg_blue * 65536);
  if (mmal_port_parameter_set(state.camera_component->control, &param.hdr) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set AWBG gains parameter." << endl;
}

void RaspiCam::CommitImageEffect() const {
  MMAL_PARAMETER_IMAGEFX_T imgFX = {{MMAL_PARAMETER_IMAGE_EFFECT, sizeof(imgFX)}, ConvertImageEffect(state.rpc_image_effect)};
  if (mmal_port_parameter_set(state.camera_component->control, &imgFX.hdr) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set image effect parameter." << endl;
}

void RaspiCam::CommitMetering() const {
  MMAL_PARAMETER_EXPOSUREMETERINGMODE_T meter_mode = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(meter_mode)}, ConvertMetering(state.rpc_exposure_meter_mode)};
  if (mmal_port_parameter_set(state.camera_component->control, &meter_mode.hdr) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set metering parameter." << endl;
}

void RaspiCam::CommitFlips() const {
  MMAL_PARAMETER_MIRROR_T mirror = {{MMAL_PARAMETER_MIRROR, sizeof(MMAL_PARAMETER_MIRROR_T)}, MMAL_PARAM_MIRROR_NONE};
  if (state.h_flip && state.v_flip)
    mirror.value = MMAL_PARAM_MIRROR_BOTH;
  else if (state.h_flip)
    mirror.value = MMAL_PARAM_MIRROR_HORIZONTAL;
  else if (state.v_flip)
    mirror.value = MMAL_PARAM_MIRROR_VERTICAL;
  if (mmal_port_parameter_set(state.camera_component->output[0], &mirror.hdr) != MMAL_SUCCESS ||
      mmal_port_parameter_set(state.camera_component->output[1], &mirror.hdr) != MMAL_SUCCESS ||
      mmal_port_parameter_set(state.camera_component->output[2], &mirror.hdr) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set horizontal/vertical flip parameter." << endl;
}

void RaspiCam::CommitVideoStabilization() const {
  if(mmal_port_parameter_set_boolean(state.camera_component->control, MMAL_PARAMETER_VIDEO_STABILISATION, state.video_stabilisation) != MMAL_SUCCESS)
    cout << __func__ << ": Failed to set video stabilization parameter." << endl;
}

void RaspiCam::SetDefaultStateParams() {

  // Default everything to zero
  memset(&state, 0, sizeof(RASPIVID_STATE));
  // should not change the state but for completeness
  state.camera_component = nullptr;
  state.framerate = 30;
  state.width = 1280;      // use a multiple of 320 (640, 1280)
  state.height = 960;		// use a multiple of 240 (480, 960)
  state.sharpness = 0;
  state.contrast = 0;
  state.brightness = 50;
  state.saturation = 0;
  state.iso = 400;
  state.video_stabilisation = false;
  state.exposure_compensation = 0;
  state.capture_fmt=RASPICAM_FORMAT_RGB;
  state.rpc_exposure_mode = RASPICAM_EXPOSURE_AUTO;
  state.rpc_exposure_meter_mode = RASPICAM_METERING_AVERAGE;
  state.rpc_awb_mode = RASPICAM_AWB_AUTO;
  state.rpc_image_effect = RASPICAM_IMAGE_EFFECT_NONE;
  state.colour_effects.enable = 0;
  state.colour_effects.u = 128;
  state.colour_effects.v = 128;
  state.rotation = 0;
  state.h_flip = state.v_flip = 0;
  state.roi.x = state.roi.y = 0.0;
  state.roi.w = state.roi.h = 1.0;
  state.shutter_speed = 0;//auto
  state.awbg_red = 1.0;
  state.awbg_blue = 1.0;

}

// TODO: why are we passing this function a pointer to a member (in all uses)?
MMAL_COMPONENT_T *RaspiCam::CreateCameraComponent(RASPIVID_STATE *state) {
  MMAL_COMPONENT_T *camera(nullptr);
  MMAL_PORT_T *video_port(nullptr);

  MMAL_STATUS_T status;
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

  if (status != MMAL_SUCCESS) {
    cerr << "Failed to create camera component";
    return nullptr;
  }
  if (!camera->output_num) {
    cerr << "Camera does not have any ports";
    mmal_component_destroy(camera);
    return nullptr;
  }

  video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];

  // configure the camera component
  // TODO: we might be able to force the sensor mode here + investigate the MMAL_PARAMETER_CAMERA_CONFIG_T type
  MMAL_PARAMETER_CAMERA_CONFIG_T camConfig;
  camConfig.hdr.id = MMAL_PARAMETER_CAMERA_CONFIG;
  camConfig.hdr.size = sizeof(camConfig);
  camConfig.max_stills_w = state->width;
  camConfig.max_stills_h = state->height;
  camConfig.stills_yuv422 = 0;
  camConfig.one_shot_stills = 0;
  camConfig.max_preview_video_w = state->width;
  camConfig.max_preview_video_h = state->height;
  camConfig.num_preview_video_frames = 3;
  camConfig.stills_capture_circular_buffer_height = 0;
  camConfig.fast_preview_resume = 0;
  // NOTE: do we want to use reset or raw?
  camConfig.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC;
  mmal_port_parameter_set(camera->control, &camConfig.hdr);

  format = video_port->format;
  // are we using the OPAQUE format here? looks like we are specifying to use RGB rather than opaque
  // so we want to change this to allow piping over to an h.264 encoder, or MJPEG, or over to an EGL via DMA
  // TODO: encoder pipeline configuration
  format->encoding_variant = ConvertFormat(state->capture_fmt);
  format->encoding = ConvertFormat(state->capture_fmt);
  format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
  format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
  // TODO: the crop parameters should be some range [0-65535], I think
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = state->width;
  format->es->video.crop.height = state->height;
  // TODO framerate is an iteger value
  format->es->video.frame_rate.num = state->framerate;
  format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

  status = mmal_port_format_commit(video_port);
  if (status /*!= MMAL_SUCCESS*/) {
    cerr << "Camera video format could not be set";
    mmal_component_destroy(camera);
    return nullptr;
  }

  // plug the callback to the video port
  // we default by connecting the video port straight to a callback function
  // TODO: deeper pipeline configuration, this requires port connection configuration
  status = mmal_port_enable(video_port, VideoBufferCallback);
  if (status) {
    cerr<< ("camera video callback error");
    // kill the camera component on a failed enable
    mmal_component_destroy (camera);
    return 0;
  }

  // Ensure there are enough buffers to avoid dropping frames
  if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  // create a pool of buffers for date flow Camera -> GPU (blocks) -> CPU
  MMAL_POOL_T *pool;
  // just use the video port recommended amount
  // note with just one component, it's slightly pointless having a pool at all I think
  // TODO: if we have additional components, we may want to change the buffer pool allocation sequence
  video_port->buffer_size = video_port->buffer_size_recommended;
  video_port->buffer_num = video_port->buffer_num_recommended;
  pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
  if (!pool) {
    cerr << "Failed to create buffer pool for video output port";
  }
  state->video_pool = pool;

  status = mmal_component_enable(camera);
  if (status /*!= MMAL_SUCCESS*/) {
    cerr << "Camera component could not be enabled";
    mmal_component_destroy(camera);
    return nullptr;
  }

  state->camera_component = camera;
  return camera;
}

void RaspiCam::DestroyCameraComponent(RASPIVID_STATE *state) {
  if (state->video_pool)
    mmal_port_pool_destroy(state->camera_component->output[MMAL_CAMERA_VIDEO_PORT], state->video_pool);
  if (state->camera_component) {
    mmal_component_destroy(state->camera_component);
    state->camera_component = nullptr;
  }
}

void RaspiCam::VideoBufferCallback(MMAL_PORT_T *port,
                                   MMAL_BUFFER_HEADER_T *buffer) {
  MMAL_BUFFER_HEADER_T *new_buffer;
  auto *p_data = (PORT_USERDATA*) port->userdata;

  bool has_grabbed = false;
  // grab the userdata mem frame
  std::unique_lock<mutex> lck(p_data->m);

  // TODO: might want to add a flag for this
  // does this callback refer to a buffer with a new frame start?
  if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_START) {
    // then compute an updated frame interval
    p_data->current_frame_interval = buffer->pts - p_data->last_pts;
    p_data->last_pts = buffer->pts;
  }
  if (p_data) {
    // we want the frame and the buffer is not empty
    if (p_data->want_to_grab && buffer->length) {
      // pin down the buffer
      mmal_buffer_header_mem_lock(buffer);

      // clear buffer and make room for new data from port callback
      p_data->_buffData.resize(buffer->length);
      memcpy(p_data->_buffData.data, buffer->data, buffer->length);
      // we have now successfully grabbed the frame so unset require flag and set completion flag
      p_data->want_to_grab = false;
      has_grabbed = true;
      // release the buffer
      mmal_buffer_header_mem_unlock(buffer);
    }
  }
  // release buffer back to the pool
  mmal_buffer_header_release(buffer);
  // assign a new buffer to the port from the pool
  if (port->is_enabled) {
    MMAL_STATUS_T  status;

    new_buffer = mmal_queue_get(p_data->pstate->video_pool->queue);

    if (new_buffer)
      status = mmal_port_send_buffer(port, new_buffer);

    if (!new_buffer || status != MMAL_SUCCESS)
      cerr << "Unable to return a buffer to the encoder port" << endl;
  }

  // force shutter speed (for some reason??)
  if (p_data->pstate->shutter_speed != 0)
    mmal_port_parameter_set_uint32(p_data->pstate->camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, p_data->pstate->shutter_speed);
  if (has_grabbed) p_data->thcond.BroadCast(); // alert blocked client
}

int RaspiCam::ConvertFormat(RASPICAM_FORMAT fmt) const {
  // TODO: do we really need the raspicam format specifier?
  switch (fmt) {
    case RASPICAM_FORMAT_RGB:return MMAL_ENCODING_RGB24;
    case RASPICAM_FORMAT_BGR:return MMAL_ENCODING_BGR24;
    case RASPICAM_FORMAT_GRAY:return MMAL_ENCODING_I420;
    case RASPICAM_FORMAT_YUV420:return MMAL_ENCODING_I420;
    default:return MMAL_ENCODING_I420;
  }
}

MMAL_PARAM_EXPOSUREMODE_T RaspiCam::ConvertExposure(RASPICAM_EXPOSURE exposure) const {
  switch (exposure) {
    case RASPICAM_EXPOSURE_OFF:return MMAL_PARAM_EXPOSUREMODE_OFF;
    case RASPICAM_EXPOSURE_AUTO:return MMAL_PARAM_EXPOSUREMODE_AUTO;
    case RASPICAM_EXPOSURE_NIGHT:return MMAL_PARAM_EXPOSUREMODE_NIGHT;
    case RASPICAM_EXPOSURE_NIGHTPREVIEW:return MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW;
    case RASPICAM_EXPOSURE_BACKLIGHT:return MMAL_PARAM_EXPOSUREMODE_BACKLIGHT;
    case RASPICAM_EXPOSURE_SPOTLIGHT:return MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT;
    case RASPICAM_EXPOSURE_SPORTS:return MMAL_PARAM_EXPOSUREMODE_SPORTS;
    case RASPICAM_EXPOSURE_SNOW:return MMAL_PARAM_EXPOSUREMODE_SNOW;
    case RASPICAM_EXPOSURE_BEACH:return MMAL_PARAM_EXPOSUREMODE_BEACH;
    case RASPICAM_EXPOSURE_VERYLONG:return MMAL_PARAM_EXPOSUREMODE_VERYLONG;
    case RASPICAM_EXPOSURE_FIXEDFPS:return MMAL_PARAM_EXPOSUREMODE_FIXEDFPS;
    case RASPICAM_EXPOSURE_ANTISHAKE:return MMAL_PARAM_EXPOSUREMODE_ANTISHAKE;
    case RASPICAM_EXPOSURE_FIREWORKS:return MMAL_PARAM_EXPOSUREMODE_FIREWORKS;
    default:return MMAL_PARAM_EXPOSUREMODE_AUTO;
  }
}

MMAL_PARAM_AWBMODE_T RaspiCam::ConvertAWB(RASPICAM_AWB awb) const {
  switch (awb) {
    case RASPICAM_AWB_OFF:return MMAL_PARAM_AWBMODE_OFF;
    case RASPICAM_AWB_AUTO:return MMAL_PARAM_AWBMODE_AUTO;
    case RASPICAM_AWB_SUNLIGHT:return MMAL_PARAM_AWBMODE_SUNLIGHT;
    case RASPICAM_AWB_CLOUDY:return MMAL_PARAM_AWBMODE_CLOUDY;
    case RASPICAM_AWB_SHADE:return MMAL_PARAM_AWBMODE_SHADE;
    case RASPICAM_AWB_TUNGSTEN:return MMAL_PARAM_AWBMODE_TUNGSTEN;
    case RASPICAM_AWB_FLUORESCENT:return MMAL_PARAM_AWBMODE_FLUORESCENT;
    case RASPICAM_AWB_INCANDESCENT:return MMAL_PARAM_AWBMODE_INCANDESCENT;
    case RASPICAM_AWB_FLASH:return MMAL_PARAM_AWBMODE_FLASH;
    case RASPICAM_AWB_HORIZON:return MMAL_PARAM_AWBMODE_HORIZON;
    default:return MMAL_PARAM_AWBMODE_AUTO;
  }
}

MMAL_PARAM_IMAGEFX_T RaspiCam::ConvertImageEffect(RASPICAM_IMAGE_EFFECT imageEffect) const {
  switch (imageEffect) {
    case RASPICAM_IMAGE_EFFECT_NONE:return MMAL_PARAM_IMAGEFX_NONE;
    case RASPICAM_IMAGE_EFFECT_NEGATIVE:return MMAL_PARAM_IMAGEFX_NEGATIVE;
    case RASPICAM_IMAGE_EFFECT_SOLARIZE:return MMAL_PARAM_IMAGEFX_SOLARIZE;
    case RASPICAM_IMAGE_EFFECT_SKETCH:return MMAL_PARAM_IMAGEFX_SKETCH;
    case RASPICAM_IMAGE_EFFECT_DENOISE:return MMAL_PARAM_IMAGEFX_DENOISE;
    case RASPICAM_IMAGE_EFFECT_EMBOSS:return MMAL_PARAM_IMAGEFX_EMBOSS;
    case RASPICAM_IMAGE_EFFECT_OILPAINT:return MMAL_PARAM_IMAGEFX_OILPAINT;
    case RASPICAM_IMAGE_EFFECT_HATCH:return MMAL_PARAM_IMAGEFX_HATCH;
    case RASPICAM_IMAGE_EFFECT_GPEN:return MMAL_PARAM_IMAGEFX_GPEN;
    case RASPICAM_IMAGE_EFFECT_PASTEL:return MMAL_PARAM_IMAGEFX_PASTEL;
    case RASPICAM_IMAGE_EFFECT_WATERCOLOR:return MMAL_PARAM_IMAGEFX_WATERCOLOUR;
    case RASPICAM_IMAGE_EFFECT_FILM:return MMAL_PARAM_IMAGEFX_FILM;
    case RASPICAM_IMAGE_EFFECT_BLUR:return MMAL_PARAM_IMAGEFX_BLUR;
    case RASPICAM_IMAGE_EFFECT_SATURATION:return MMAL_PARAM_IMAGEFX_SATURATION;
    case RASPICAM_IMAGE_EFFECT_COLORSWAP:return MMAL_PARAM_IMAGEFX_COLOURSWAP;
    case RASPICAM_IMAGE_EFFECT_WASHEDOUT:return MMAL_PARAM_IMAGEFX_WASHEDOUT;
    case RASPICAM_IMAGE_EFFECT_POSTERISE:return MMAL_PARAM_IMAGEFX_POSTERISE;
    case RASPICAM_IMAGE_EFFECT_COLORPOINT:return MMAL_PARAM_IMAGEFX_COLOURPOINT;
    case RASPICAM_IMAGE_EFFECT_COLORBALANCE:return MMAL_PARAM_IMAGEFX_COLOURBALANCE;
    case RASPICAM_IMAGE_EFFECT_CARTOON:return MMAL_PARAM_IMAGEFX_CARTOON;
    default:return MMAL_PARAM_IMAGEFX_NONE;
  }
}

MMAL_PARAM_EXPOSUREMETERINGMODE_T RaspiCam::ConvertMetering(RASPICAM_METERING metering) const {
  switch (metering){
    case RASPICAM_METERING_AVERAGE:
      return MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
    case RASPICAM_METERING_SPOT:
      return  MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT;
    case RASPICAM_METERING_BACKLIT:
      return MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT;
    case RASPICAM_METERING_MATRIX:
      return MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX;
    default:
      return MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
  }
}


}

