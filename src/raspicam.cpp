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

#include <iostream>
#include <cstdio>

#include "mmal/util/mmal_util.h"
#include "mmal/util/mmal_util_params.h"
#include "mmal/util/mmal_default_components.h"

#include "raspicam.h"

using namespace std;
namespace raspicam {

#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2
#define VIDEO_FRAME_RATE_DEN 1
#define VIDEO_OUTPUT_BUFFERS_NUM 3

    RaspiCam::RaspiCam() {
        cameraVideoPort = nullptr;
        isOpen = false;
        isCapturing = false;
        SetDefaultStateParams();
    }

    RaspiCam::~RaspiCam() {
        Release();
    }

    bool RaspiCam::Open(bool startCapture) {

        if (isOpen) return false;

        // create camera
        if (!CreateCameraComponent(&state)) {
            cerr << __func__ << "Failed to create camera component" << __FILE__ << " " << __LINE__ << endl;
            return false;
        }
        CommitParameters();
        cameraVideoPort = state.cameraComponent->output[MMAL_CAMERA_VIDEO_PORT];
        callbackData.pstate = &state;

        // assign data to use for callback
        // TODO: understand better the flow of pot.userdata <-> this.callbackData
        cameraVideoPort->userdata = (struct MMAL_PORT_USERDATA_T *) &callbackData;

        isOpen = true;
        // TODO: bad variable naming
        if (startCapture) return StartCapture();
    }

    bool RaspiCam::StartCapture() {
        if (!isOpen) {
            cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << "not opened." << endl;
            return false;
        }

        // start capture
        if (mmal_port_parameter_set_boolean(cameraVideoPort, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
            Release();
            return false;
        }

        // send all the buffers to the video port
        unsigned int num = mmal_queue_length(state.videoPool->queue);
        int q;
        for (q = 0; q < num; q++) {
            MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.videoPool->queue);

            if (!buffer)
                cerr << "Unable to get required buffer " << q << " from pool queue" << endl;

            if (mmal_port_send_buffer(cameraVideoPort, buffer) != MMAL_SUCCESS)
                cerr << "Unable to send a buffer to encoder output port " << q << endl;
        }
        isCapturing = true;
        return true;
    }

    void RaspiCam::Release() {
        if (!isOpen) return;

        // disable cameraVideoPort
        if (cameraVideoPort && cameraVideoPort->is_enabled) {
            mmal_port_disable(cameraVideoPort);
            cameraVideoPort = nullptr;
        }
        // disable all our ports that are not handled by connections
        // TODO: this might not be correct, see again
        if (state.cameraComponent)
            mmal_component_disable(state.cameraComponent);

        DestroyCameraComponent(&state);

        isOpen = false;
        isCapturing = false;
    }

    bool RaspiCam::Grab() {
        if (!isCapturing) return false;
        callbackData.WaitForFrame();
        return true;
    }

    void RaspiCam::Retrieve(unsigned char *data, RASPICAM_FORMAT type) {
        if (callbackData._buffData.size == 0) return;
        // TODO: this is you wot m7!?
        if (type != RASPICAM_FORMAT_IGNORE) {
            cerr << __FILE__ << ":" << __LINE__ << " : Retrieve type is not RASPICAM_FORMAT_IGNORE as expected" << endl;
        }
        if (state.captureFmt == RASPICAM_FORMAT_YUV420) {
            auto imagePtr = callbackData._buffData.data;
            for (int i = 0; i < state.height + state.height / 2; i++) {
                memcpy(data, imagePtr, state.width);
                data += state.width;
                imagePtr += format->es->video.width; // line stride
            }
        } else if (state.captureFmt == RASPICAM_FORMAT_GRAY) {
            auto imagePtr = callbackData._buffData.data;
            for (int i = 0; i < state.height; i++) {
                memcpy(data, imagePtr, state.width);
                data += state.width;
                imagePtr += format->es->video.width; // line stride
            }
        } else if (state.captureFmt == RASPICAM_FORMAT_BGR || state.captureFmt == RASPICAM_FORMAT_RGB) {
            auto imagePtr = callbackData._buffData.data;
            for (int i = 0; i < state.height; i++) {
                memcpy(data, imagePtr, state.width * 3);
                data += state.width * 3;
                imagePtr += format->es->video.width * 3; // line stride
            }
        }
    }

    unsigned char *RaspiCam::GetImageBufferData() const {
        return callbackData._buffData.data;
    }

    size_t RaspiCam::GetImageBufferSize() const {
        return GetImageTypeSize(GetFormat());
    }

    size_t RaspiCam::GetImageTypeSize(RASPICAM_FORMAT type) const {
        switch (type) {
            case RASPICAM_FORMAT_YUV420:
                return GetWidth() * GetHeight() + 2 * ((GetWidth() / 2 * GetHeight() / 2));
                break;
            case RASPICAM_FORMAT_GRAY:
                return GetWidth() * GetHeight();
                break;
            case RASPICAM_FORMAT_BGR:
            case RASPICAM_FORMAT_RGB:
                return 3 * GetWidth() * GetHeight();
                break;
            default:
                return 0;
        }
    }


    void RaspiCam::SetFormat(RASPICAM_FORMAT fmt) {
        if (isOpen) {
            cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
                 << ": cannot change format with camera already open" << endl;
            return;
        }
        state.captureFmt = fmt;
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
        if (isOpen) CommitBrightness();
    }

    void RaspiCam::SetRotation(int rotation) {
        while (rotation < 0) rotation += 360;
        if (rotation >= 360) rotation %= 360;
        state.rotation = rotation;
        if (isOpen) CommitRotation();
    }

    void RaspiCam::SetISO(int iso) {
        state.iso = iso;
        if (isOpen) CommitISO();
    }

    void RaspiCam::SetSharpness(int sharpness) {
        if (sharpness < -100) sharpness = -100;
        if (sharpness > 100) sharpness = 100;
        state.sharpness = sharpness;
        if (isOpen) CommitSharpness();
    }

    void RaspiCam::SetContrast(int contrast) {
        if (contrast < -100) contrast = -100;
        if (contrast > 100) contrast = 100;
        state.contrast = contrast;
        if (isOpen) CommitContrast();
    }

    void RaspiCam::SetSaturation(int saturation) {
        if (saturation < -100) saturation = -100;
        if (saturation > 100) saturation = 100;
        state.saturation = saturation;
        if (isOpen) CommitSaturation();
    }

    void RaspiCam::SetExposure(RASPICAM_EXPOSURE exposure) {
        state.rpcExposureMode = exposure;
        if (isOpen) CommitExposure();
    }

    void RaspiCam::SetShutterSpeed(unsigned int ss) {
        // TODO: I don't know what orifice they pulled this max value out of
        if (ss > 330000) ss = 330000;
        state.shutterSpeed = ss;
        if (isOpen) CommitShutterSpeed();
    }

    void RaspiCam::SetVideoStabilization(bool v) {
        state.videoStabilisation = v;
        if (isOpen) CommitVideoStabilization();
    }

    void RaspiCam::SetExposureCompensation(int val) {
        if (val < -10) val = -10;
        if (val > 10) val = 10;
        state.exposureCompensation = val;
        if (isOpen) CommitExposureCompensation();
    }

    void RaspiCam::SetAWB(RASPICAM_AWB awb) {
        state.rpcAwbMode = awb;
        if (isOpen) CommitAWB();
    }

    void RaspiCam::SetAWB_RB(float r, float b) {
        state.awbgRed = r;
        state.awbgBlue = b;
        if (isOpen) CommitAWB_RB();
    }

    void RaspiCam::SetImageEffect(RASPICAM_IMAGE_EFFECT imageEffect) {
        state.rpcImageEffect = imageEffect;
        if (isOpen) CommitImageEffect();
    }

    void RaspiCam::SetMetering(RASPICAM_METERING metering) {
        state.rpcExposureMeterMode = metering;
        if (isOpen) CommitMetering();
    }

    void RaspiCam::SetHorizontalFlip(bool hFlip) {
        state.hFlip = hFlip;
        if (isOpen) CommitFlips();
    }

    void RaspiCam::SetVerticalFlip(bool vFlip) {
        state.hFlip = vFlip;
        if (isOpen) CommitFlips();
    }

    void RaspiCam::SetFrameRate(unsigned int fr) {
        state.framerate = fr;
    }

// here we are assuming that the framerate is always just an integer value, so check VIDEO_FRAMRE_RATE_DEN is def as 1
#if VIDEO_FRAME_RATE_DEN != 1
#   error "fix framerate setting function to account for the different denominator"
#endif

    void RaspiCam::SetFramerateDelta(MMAL_RATIONAL_T setpoint) {
        if (!isOpen) {
            cout << __func__ << ": Framerate Delta can only be set when the camera is open" << endl;
        }
        // first get the new framerate required
        setpoint.num += state.framerate * setpoint.den;
        if (mmal_port_parameter_set_rational(state.cameraComponent->output[MMAL_CAMERA_VIDEO_PORT],
                                             MMAL_PARAMETER_FRAME_RATE, setpoint) != MMAL_SUCCESS)
            cout << __func__ << ": Failed to set framerate Delta parameter." << endl;
    }


    RASPICAM_FORMAT RaspiCam::GetFormat() const { return _impl->getFormat(); }

    unsigned int RaspiCam::GetWidth() const { return _impl->getWidth(); }

    unsigned int RaspiCam::GetHeight() const { return _impl->getHeight(); }

    unsigned int RaspiCam::GetBrightness() const { return _impl->getBrightness(); }

    unsigned int RaspiCam::GetRotation() const { return _impl->getRotation(); }

    int RaspiCam::GetISO() const { return _impl->getISO(); }

    unsigned int
    RaspiCam::GetShutterSpeed() const { return _impl->getShutterSpeed(); }//return _impl->getShutterSpeed();}
    unsigned int RaspiCam::GetFrameRate() const { return _impl->getFrameRate(); }

    void RaspiCam::GetFramerateDelta(MMAL_RATIONAL_T *value) const {
        // TODO: need to check the camera is currently active?
        // TODO: something about if there is no encoder active use preview port rather than video
        // first read the actual framerate parameter into value
        mmal_port_parameter_get_rational(state.cameraComponent->output[MMAL_CAMERA_VIDEO_PORT],
                                         MMAL_PARAMETER_FRAME_RATE, value);


        // remove the initialised format framerate from the current value
        value->num -= state.framerate * value->den;
    }

    int RaspiCam::GetSharpness() const { return _impl->getSharpness(); }

    int RaspiCam::GetContrast() const { return _impl->getContrast(); }

    int RaspiCam::GetSaturation() const { return _impl->getSaturation(); }

    RASPICAM_EXPOSURE RaspiCam::GetExposure() const { return _impl->getExposure(); }

    RASPICAM_AWB RaspiCam::GetAWB() const { return _impl->getAWB(); }

    float RaspiCam::GetAWBG_red() const { return _impl->getAWBG_red(); }

    float RaspiCam::GetAWBG_blue() const { return _impl->getAWBG_blue(); }

    RASPICAM_IMAGE_EFFECT RaspiCam::GetImageEffect() const { return _impl->getImageEffect(); };

    RASPICAM_METERING RaspiCam::GetMetering() const { return _impl->getMetering(); }

    bool RaspiCam::IsHorizontallyFlipped() const { return _impl->isHorizontallyFlipped(); }

    bool RaspiCam::IsVerticallyFlipped() const { return _impl->isVerticallyFlipped(); }

    //Returns an id of the camera. We assume the camera id is the one of the raspberry
    //the id is obtained using raspberry serial number obtained in /proc/cpuinfo
    std::string RaspiCam::GetId() const { return _impl->getId(); }

    void RaspiCam::CommitParameters() {
        assert(state.cameraComponent != nullptr);
        CommitSaturation();
        CommitSharpness();
        CommitContrast();
        CommitBrightness();
        CommitISO();
        if (state.shutterSpeed!=0){
            CommitShutterSpeed();
            state.rpcExposureMode=RASPICAM_EXPOSURE_FIXEDFPS;
            CommitExposure();
        } else CommitExposure();
        CommitExposureCompensation();
        CommitMetering();
        CommitImageEffect();
        CommitRotation();
        CommitFlips();
        CommitVideoStabilization();
        CommitAWB();
        CommitAWB_RB();
    }

    // TODO: why are we passing a pointer to a member??
    MMAL_COMPONENT_T *RaspiCam::CreateCameraComponent(RASPIVID_STATE *state) {
        MMAL_COMPONENT_T *camera(nullptr);
        MMAL_PORT_T *videoPort(nullptr);

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

        videoPort = camera->output[MMAL_CAMERA_VIDEO_PORT];

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


        format = videoPort->format;
        // are we using the OPAQUE format here? looks like we are specifying to use RGB rather than opaque
        // so we want to change this to allow piping over to an h.264 encoder, or MJPEG, or over to an EGL via DMA
        // TODO: encoder pipeline configuration
        format->encoding_variant = ConvertFormat(state->captureFmt);
        format->encoding = ConvertFormat(state->captureFmt);
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

        status = mmal_port_format_commit(videoPort);
        if (status /*!= MMAL_SUCCESS*/) {
            cerr << "Camera video format could not be set";
            mmal_component_destroy(camera);
            return nullptr;
        }

        // create a pool of buffers for date flow Camera -> GPU (blocks) -> CPU
        MMAL_POOL_T *pool;
        // just use the video port recommended amount
        // note with just one component, it's slightly pointless having a pool at all I think
        // TODO: if we have additional components, we may want to change the buffer pool allocation sequence
        videoPort->buffer_size = videoPort->buffer_size_recommended;
        videoPort->buffer_num = videoPort->buffer_num_recommended;
        pool = mmal_port_pool_create(videoPort, videoPort->buffer_num, videoPort->buffer_size);
        if (!pool) {
            cerr << "Failed to create buffer pool for video output port";
        }
        state->videoPool = pool;


        status = mmal_component_enable(camera);
        if (status /*!= MMAL_SUCCESS*/) {
            cerr << "Camera component could not be enabled";
            mmal_component_destroy(camera);
            return nullptr;
        }

        state->cameraComponent = camera;
        return camera;
    }

    void RaspiCam::DestroyCameraComponent(RASPIVID_STATE *state) {
        if (state->videoPool)
            mmal_port_pool_destroy(state->cameraComponent->output[MMAL_CAMERA_VIDEO_PORT], state->videoPool);
        if (state->cameraComponent) {
            mmal_component_destroy(state->cameraComponent);
            state->cameraComponent = nullptr;
        }
    }

    int RaspiCam::ConvertFormat(RASPICAM_FORMAT fmt) {
        // TODO: do we really need the raspicam format specifier?
        switch (fmt) {
            case RASPICAM_FORMAT_RGB:
                return MMAL_ENCODING_RGB24;
            case RASPICAM_FORMAT_BGR:
                return MMAL_ENCODING_BGR24;
            case RASPICAM_FORMAT_GRAY:
                return MMAL_ENCODING_I420;
            case RASPICAM_FORMAT_YUV420:
                return MMAL_ENCODING_I420;
            default:
                return MMAL_ENCODING_I420;
        }
    }

    MMAL_PARAM_EXPOSUREMODE_T RaspiCam::ConvertExposure(RASPICAM_EXPOSURE exposure) {
        switch (exposure) {
            case RASPICAM_EXPOSURE_OFF:
                return MMAL_PARAM_EXPOSUREMODE_OFF;
            case RASPICAM_EXPOSURE_AUTO:
                return MMAL_PARAM_EXPOSUREMODE_AUTO;
            case RASPICAM_EXPOSURE_NIGHT:
                return MMAL_PARAM_EXPOSUREMODE_NIGHT;
            case RASPICAM_EXPOSURE_NIGHTPREVIEW:
                return MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW;
            case RASPICAM_EXPOSURE_BACKLIGHT:
                return MMAL_PARAM_EXPOSUREMODE_BACKLIGHT;
            case RASPICAM_EXPOSURE_SPOTLIGHT:
                return MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT;
            case RASPICAM_EXPOSURE_SPORTS:
                return MMAL_PARAM_EXPOSUREMODE_SPORTS;
            case RASPICAM_EXPOSURE_SNOW:
                return MMAL_PARAM_EXPOSUREMODE_SNOW;
            case RASPICAM_EXPOSURE_BEACH:
                return MMAL_PARAM_EXPOSUREMODE_BEACH;
            case RASPICAM_EXPOSURE_VERYLONG:
                return MMAL_PARAM_EXPOSUREMODE_VERYLONG;
            case RASPICAM_EXPOSURE_FIXEDFPS:
                return MMAL_PARAM_EXPOSUREMODE_FIXEDFPS;
            case RASPICAM_EXPOSURE_ANTISHAKE:
                return MMAL_PARAM_EXPOSUREMODE_ANTISHAKE;
            case RASPICAM_EXPOSURE_FIREWORKS:
                return MMAL_PARAM_EXPOSUREMODE_FIREWORKS;
            default:
                return MMAL_PARAM_EXPOSUREMODE_AUTO;
        }
    }

    MMAL_PARAM_AWBMODE_T RaspiCam::ConvertAWB(RASPICAM_AWB awb) {
        switch (awb) {
            case RASPICAM_AWB_OFF:
                return MMAL_PARAM_AWBMODE_OFF;
            case RASPICAM_AWB_AUTO:
                return MMAL_PARAM_AWBMODE_AUTO;
            case RASPICAM_AWB_SUNLIGHT:
                return MMAL_PARAM_AWBMODE_SUNLIGHT;
            case RASPICAM_AWB_CLOUDY:
                return MMAL_PARAM_AWBMODE_CLOUDY;
            case RASPICAM_AWB_SHADE:
                return MMAL_PARAM_AWBMODE_SHADE;
            case RASPICAM_AWB_TUNGSTEN:
                return MMAL_PARAM_AWBMODE_TUNGSTEN;
            case RASPICAM_AWB_FLUORESCENT:
                return MMAL_PARAM_AWBMODE_FLUORESCENT;
            case RASPICAM_AWB_INCANDESCENT:
                return MMAL_PARAM_AWBMODE_INCANDESCENT;
            case RASPICAM_AWB_FLASH:
                return MMAL_PARAM_AWBMODE_FLASH;
            case RASPICAM_AWB_HORIZON:
                return MMAL_PARAM_AWBMODE_HORIZON;
            default:
                return MMAL_PARAM_AWBMODE_AUTO;
        }
    }

    MMAL_PARAM_IMAGEFX_T RaspiCam::ConvertImageEffect(RASPICAM_IMAGE_EFFECT imageEffect) {
        switch (imageEffect) {
            case RASPICAM_IMAGE_EFFECT_NONE:
                return MMAL_PARAM_IMAGEFX_NONE;
            case RASPICAM_IMAGE_EFFECT_NEGATIVE:
                return MMAL_PARAM_IMAGEFX_NEGATIVE;
            case RASPICAM_IMAGE_EFFECT_SOLARIZE:
                return MMAL_PARAM_IMAGEFX_SOLARIZE;
            case RASPICAM_IMAGE_EFFECT_SKETCH:
                return MMAL_PARAM_IMAGEFX_SKETCH;
            case RASPICAM_IMAGE_EFFECT_DENOISE:
                return MMAL_PARAM_IMAGEFX_DENOISE;
            case RASPICAM_IMAGE_EFFECT_EMBOSS:
                return MMAL_PARAM_IMAGEFX_EMBOSS;
            case RASPICAM_IMAGE_EFFECT_OILPAINT:
                return MMAL_PARAM_IMAGEFX_OILPAINT;
            case RASPICAM_IMAGE_EFFECT_HATCH:
                return MMAL_PARAM_IMAGEFX_HATCH;
            case RASPICAM_IMAGE_EFFECT_GPEN:
                return MMAL_PARAM_IMAGEFX_GPEN;
            case RASPICAM_IMAGE_EFFECT_PASTEL:
                return MMAL_PARAM_IMAGEFX_PASTEL;
            case RASPICAM_IMAGE_EFFECT_WATERCOLOR:
                return MMAL_PARAM_IMAGEFX_WATERCOLOUR;
            case RASPICAM_IMAGE_EFFECT_FILM:
                return MMAL_PARAM_IMAGEFX_FILM;
            case RASPICAM_IMAGE_EFFECT_BLUR:
                return MMAL_PARAM_IMAGEFX_BLUR;
            case RASPICAM_IMAGE_EFFECT_SATURATION:
                return MMAL_PARAM_IMAGEFX_SATURATION;
            case RASPICAM_IMAGE_EFFECT_COLORSWAP:
                return MMAL_PARAM_IMAGEFX_COLOURSWAP;
            case RASPICAM_IMAGE_EFFECT_WASHEDOUT:
                return MMAL_PARAM_IMAGEFX_WASHEDOUT;
            case RASPICAM_IMAGE_EFFECT_POSTERISE:
                return MMAL_PARAM_IMAGEFX_POSTERISE;
            case RASPICAM_IMAGE_EFFECT_COLORPOINT:
                return MMAL_PARAM_IMAGEFX_COLOURPOINT;
            case RASPICAM_IMAGE_EFFECT_COLORBALANCE:
                return MMAL_PARAM_IMAGEFX_COLOURBALANCE;
            case RASPICAM_IMAGE_EFFECT_CARTOON:
                return MMAL_PARAM_IMAGEFX_CARTOON;
            default:
                return MMAL_PARAM_IMAGEFX_NONE;
        }
    }
}

