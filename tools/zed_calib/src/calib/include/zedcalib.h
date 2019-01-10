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

#include <ros/ros.h>
#include <ros/timer.h>
#include <dynamic_reconfigure/server.h>
#include "zed_calib/zed_calibConfig.h"

#include "zed_calib/calib_stamped.h"

#include "sl/Camera.hpp"

#include "csmartmean.h"

namespace zed_calib {

    class ZedCalib {
      public:
        ZedCalib(ros::NodeHandle nh, ros::NodeHandle priv_nh);
        virtual ~ZedCalib();

        // Initialization
        bool init();

      protected:
        // Data acquisition timer callback
        void timerCallback(const ros::TimerEvent& e);

        // Callback to handle dynamic reconfigure events in ROS
        void dynamicReconfCallback(zed_calib::zed_calibConfig& cfg, uint32_t level);

      private:
        // Node Handles
        ros::NodeHandle mNh;
        ros::NodeHandle mPrivNh;

        // ZED Params
        sl::InitParameters mZedParams;

        // Data acquisition timer
        ros::Timer mCalibTimer;

        // Parameters
        int mCamFreq = 30;
        sl::RESOLUTION mCamRes = sl::RESOLUTION_HD720;
        bool mCamFlip = false;

        // Dynamic reconfigure
        boost::shared_ptr<dynamic_reconfigure::Server<zed_calib::zed_calibConfig>> mDynRecServer;

        // Dynamic parameters
        int mMeanWinSize = 30;
        double mDesRoll = 0.0;
        double mDesPitch = 0.0;
        double mDesHeight = 0.0;

        // ZED Camera
        sl::Camera mZed;

        // Message publisher
        ros::Publisher mCalibPub;

        // Message
        zed_calib::calib_stampedPtr mCalibMsgPtr;

        // Mean values
        CSmartMean mMeanRoll;
        CSmartMean mMeanPitch;
        CSmartMean mMeanHeight;
    };

} // namespace zed_calib
