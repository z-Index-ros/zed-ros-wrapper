// /////////////////////////////////////////////////////////////////////////

//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// /////////////////////////////////////////////////////////////////////////

#include "zedcalib.h"

#include "zed_calib/reset_means.h"
#include "zed_calib/zed_calibConfig.h"

namespace zed_calib {

    ZedCalib::ZedCalib(ros::NodeHandle nh, ros::NodeHandle priv_nh) {
        mNh = nh;
        mPrivNh = priv_nh;
    }

    ZedCalib::~ZedCalib() {
        if (mZed.isOpened()) {
            mZed.close();
        }
    }

    bool ZedCalib::init() {
        // Load parameters
        int resol;

        if (mPrivNh.getParam("resolution", resol)) {
            mCamRes = static_cast<sl::RESOLUTION>(resol);
        } else {
            ROS_WARN("No 'resolution' param. Using the default value");
            mCamRes = sl::RESOLUTION_HD720;
        }

        ROS_INFO_STREAM("Resolution: " << sl::toString(mCamRes).c_str());

        if (!mPrivNh.getParam("framerate", mCamFreq)) {
            ROS_WARN("No 'framerate' param. Using the default value");
            mCamFreq = 15;
        }

        ROS_INFO_STREAM("Framerate: " << mCamFreq);

        if (!mPrivNh.getParam("camera_flip", mCamFlip)) {
            ROS_WARN("No 'camera_flip' param. Using the default value");
            mCamFlip = false;
        }

        ROS_INFO_STREAM("Camera flipped: " << (mCamFlip ? "TRUE" : "FALSE"));

        // ZED params
        mZedParams.camera_fps = mCamFreq;
        mZedParams.camera_resolution = mCamRes;
        mZedParams.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;
        mZedParams.coordinate_units = sl::UNIT_METER;
        mZedParams.depth_mode = sl::DEPTH_MODE_ULTRA;
        mZedParams.sdk_verbose = true;

        // Camera opening
        sl::ERROR_CODE connStatus;

        int errorCount = 0;

        while (connStatus != sl::SUCCESS) {
            connStatus = mZed.open(mZedParams);
            ROS_INFO_STREAM(toString(connStatus));
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            if (++errorCount >= 15) {
                ROS_ERROR("Cannot connect to a ZED camera");
                return false;
            }

            if (!mPrivNh.ok()) {
                ROS_ERROR("User pressed Ctrl+C ... ");
                mZed.close();
                return false;
            }
        }

        sl::CameraInformation camInfo = mZed.getCameraInformation();

        ROS_INFO_STREAM(sl::toString(camInfo.camera_model).c_str() << " connected");
        ROS_INFO_STREAM("Serial number: " << camInfo.serial_number);
        ROS_INFO_STREAM("Firmare: " << camInfo.firmware_version);

        // Publisher
        mCalibPub = mPrivNh.advertise<zed_calib::calib_stamped>("calib_data", 1);

        // Message
        mCalibMsgPtr.reset(new zed_calib::calib_stamped);

        // Grab timer
        mCalibTimer = mPrivNh.createTimer(ros::Duration(1.0 / mCamFreq), &ZedCalib::timerCallback, this);

        return true;
    }

    void ZedCalib::timerCallback(const ros::TimerEvent& e) {

        if (!mZed.isOpened()) {
            ROS_WARN_STREAM_THROTTLE(1.0, "ZED camera not opened");
            return;
        }

        sl::ERROR_CODE grabStatus = mZed.grab();

        if (grabStatus != sl::SUCCESS) {
            return;    // TODO handle camera errors!!!
        }

        // Message Initialization
        mCalibMsgPtr->header.stamp = ros::Time::now();
        mCalibMsgPtr->header.frame_id = "zed_camera_center";
        mCalibMsgPtr->height = -1.0f;
        mCalibMsgPtr->roll = 0.0f;
        mCalibMsgPtr->pitch = 0.0f;

        mCalibPub.publish(mCalibMsgPtr);

    }

} // namespace zed_calib
