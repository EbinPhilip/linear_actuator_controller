#ifndef __LINEAR_ACTUATOR_CONTROLLER_H__
#define __LINEAR_ACTUATOR_CONTROLLER_H__

#include <configurable_control_hw/Actuator_Controller.h>
#include "Linear_Actuator_Properties.h"
#include "linear_actuator_controller/LinearActuatorFeedback.h"
#include "linear_actuator_controller/LinearActuatorInput.h"

#include <ros/ros.h>

#include <map>
#include <string>
#include <memory>
#include <thread>

namespace Linear_Actuator
{

    class Linear_Actuator_Controller : public Actuator_Controller
    {
    public:
        Linear_Actuator_Controller(const std::string &controller_name, const std::string &command_topic,
                                   const std::string &feedback_topic, double publish_hz,
                                   int max_position, float max_speed, float max_acceleration, bool &stop_flag);
        ~Linear_Actuator_Controller();

        void setActuator(Linear_Actuator::Actuator_Properties_Ptr actuator);

        virtual void readState() override;
        virtual void writeCommand() override;

        virtual void enableActuators() override;
        virtual void disableActuators() override;

        virtual bool getErrorDetails(std::string &error_msg) override;

        virtual ::Actuator_Properties_Ptr getActuator(const std::string &) override;
        virtual void getActuatorNames(std::vector<std::string> &) override;

        Actuator_Properties_Ptr getActuator();

    protected:
        void _actuatorMsgCB(const linear_actuator_controller::LinearActuatorFeedback &);
        void _publishCommand();
        bool _checkConnection();

        std::string controller_name_;
        std::string command_topic_;
        std::string feedback_topic_;

        double publish_hz_;

        bool error_status_;
        std::string error_msg_;
        bool &stop_flag_;

        bool actuator_status_;

        linear_actuator_controller::LinearActuatorFeedback feedback_;
        linear_actuator_controller::LinearActuatorInput input_;

        ros::Subscriber actuator_sub_;
        ros::Publisher actuator_pub_;
        bool sub_started_;
        
        std::thread publish_thread_;
        bool stop_publisher_;

        ros::Time last_cb_;

        ros::NodeHandle nh_;
        ros::AsyncSpinner spinner_;

        Actuator_Properties_Ptr actuator_;
    };
}
#endif