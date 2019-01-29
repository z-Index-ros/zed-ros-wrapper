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

#include <tf2/LinearMath/Transform.h>

#include "zed_calib/zed_calibConfig.h"

namespace zed_calib {

#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

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

        if (!mPrivNh.getParam("mean_win_size", mMeanWinSize)) {
            ROS_WARN("No 'mMeanWinSize' param. Using the default value");
            mMeanWinSize = 30;
        }

        ROS_INFO_STREAM("Mobile mean win size: " << mMeanWinSize);

        // Mean variables
        mMeanHeight.setWinSize(mMeanWinSize);
        mMeanRoll.setWinSize(mMeanWinSize);
        mMeanPitch.setWinSize(mMeanWinSize);


        // Dynamic Reconfigure parameters
        mDynRecServer = boost::make_shared<dynamic_reconfigure::Server<zed_calib::zed_calibConfig>>();
        dynamic_reconfigure::Server<zed_calib::zed_calibConfig>::CallbackType f;
        f = boost::bind(&ZedCalib::dynamicReconfCallback, this, _1, _2);
        mDynRecServer->setCallback(f);

        // ZED params
        mZedParams.camera_fps = mCamFreq;
        mZedParams.camera_resolution = mCamRes;
        mZedParams.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;
        mZedParams.coordinate_units = sl::UNIT_METER;
        mZedParams.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
        mZedParams.sdk_verbose = true;

        // Camera opening
        sl::ERROR_CODE connStatus;

        int errorCount = 0;

        while (1) {
            connStatus = mZed.open(mZedParams);
            ROS_INFO_STREAM(toString(connStatus));

            if (connStatus == sl::SUCCESS) {
                break;
            }

            if (++errorCount >= 30) {
                ROS_ERROR("Cannot connect to a ZED camera");
                return false;
            }

            if (!mPrivNh.ok()) {
                ROS_ERROR("User pressed Ctrl+C ... ");
                mZed.close();
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        sl::CameraInformation camInfo = mZed.getCameraInformation();

        // Enable tracking
        mZed.enableTracking();

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

        // Camera info
        sl::CameraInformation camInfo = mZed.getCameraInformation();

        // Runtime parameters
        sl::RuntimeParameters runtimeParams;
        runtimeParams.enable_depth = true;
        runtimeParams.measure3D_reference_frame = sl::REFERENCE_FRAME_CAMERA;
        runtimeParams.sensing_mode = sl::SENSING_MODE_STANDARD;

        sl::ERROR_CODE grabStatus = mZed.grab(runtimeParams);

        if (grabStatus != sl::SUCCESS) {
            if (grabStatus != sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                ROS_WARN_STREAM("Camera error: " << sl::toString(grabStatus));
            }

            return;    // TODO handle camera errors!!!
        }

        double roll, pitch;
        double height = -1.0;

        // IMU orientation
        if (camInfo.camera_model == sl::MODEL_ZED_M) {
            sl::IMUData imu_data;
            mZed.getIMUData(imu_data, sl::TIME_REFERENCE_IMAGE);

            // Roll & Pitch from IMU
            double imuRoll, imuPitch, imuYaw;
            sl::Orientation quat = imu_data.getOrientation();
            tf2::Matrix3x3(tf2::Quaternion(quat.ox, quat.oy, quat.oz, quat.ow)).getRPY(imuRoll, imuPitch, imuYaw);

            roll = imuRoll * RAD2DEG;
            pitch = imuPitch * RAD2DEG;

            ROS_DEBUG("IMU - Roll: %.2f - Pitch: %.2f", roll, pitch);

            // Mean update
            mMeanRoll.addValue(roll);
            mMeanPitch.addValue(pitch);
        }

        // Floor Height
        sl::Plane floorPlane;
        sl::Transform cameraTransform;

        bool floorDetected = false;
        sl::ERROR_CODE floorErr = mZed.findFloorPlane(floorPlane, cameraTransform);

        if (floorErr == sl::SUCCESS) {

            // Roll & Pitch from floor plane

            double floorRoll, floorPitch, floorYaw;
            sl::Orientation quat = cameraTransform.getOrientation();
            tf2::Matrix3x3(tf2::Quaternion(quat.ox, quat.oy, quat.oz, quat.ow)).getRPY(floorRoll, floorPitch, floorYaw);

            roll = floorRoll * RAD2DEG;
            pitch = floorPitch * RAD2DEG;

            ROS_DEBUG("FLOOR - Roll: %.2f - Pitch: %.2f", roll,
                      pitch); // NOTE:  This is valid only when the v2.8 code will be updated by Pierre

            if (camInfo.camera_model == sl::MODEL_ZED) {
                // Mean update
                mMeanRoll.addValue(roll);
                mMeanPitch.addValue(pitch);
            }

            height = cameraTransform.getTranslation().tz;

            ROS_DEBUG("FLOOR - Height: %.2f m", height);

            mMeanHeight.addValue(height);
            floorDetected = true;
        } else {
            ROS_WARN_STREAM("Floor detection:" << sl::toString(floorErr).c_str());
        }


        // Message Initialization
        mCalibMsgPtr->header.stamp = ros::Time::now();
        mCalibMsgPtr->header.frame_id = "zed_camera_center";

        // Message Publishing
        mCalibMsgPtr->roll = mMeanRoll.getMean();
        mCalibMsgPtr->pitch = mMeanPitch.getMean();
        mCalibMsgPtr->height = floorDetected ? mMeanHeight.getMean() : -1.0;
        mCalibMsgPtr->des_roll = mDesRoll;
        mCalibMsgPtr->des_pitch = mDesPitch;
        mCalibMsgPtr->des_height = mDesHeight;

        mCalibPub.publish(mCalibMsgPtr);
    }

    void ZedCalib::dynamicReconfCallback(zed_calib::zed_calibConfig& cfg, uint32_t level) {
        switch (level) {
        case 0:
            mMeanWinSize = cfg.mean_win_size;
            ROS_INFO_STREAM("Reconfigure mean_win_size: " << mMeanWinSize);

            mMeanHeight.setWinSize(mMeanWinSize);
            mMeanRoll.setWinSize(mMeanWinSize);
            mMeanPitch.setWinSize(mMeanWinSize);

            break;

        case 1:
            mDesRoll = cfg.desired_roll;
            ROS_INFO_STREAM("Reconfigure desired_roll: " << mDesRoll);
            break;

        case 2:
            mDesPitch = cfg.desired_pitch;
            ROS_INFO_STREAM("Reconfigure desired_pitch: " << mDesPitch);
            break;

        case 3:
            mDesHeight = cfg.desired_height;
            ROS_INFO_STREAM("Reconfigure desired_height: " << mDesHeight);
            break;
        }
    }

} // namespace zed_calib
