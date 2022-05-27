#include "Linear_Actuator_Controller.h"

#include <rotational_units/Rotational_Units.h>

#include <ros/console.h>

#include <stdexcept>
#include <cmath>

using namespace Linear_Actuator;

Linear_Actuator_Controller::Linear_Actuator_Controller(const std::string &controller_name,
                                                       const std::string &command_topic, const std::string &feedback_topic, double publish_hz,
                                                       int max_position, float max_speed, float max_acceleration, bool &stop_flag)
    : controller_name_(controller_name),
      command_topic_(command_topic),
      feedback_topic_(feedback_topic),
      publish_hz_(publish_hz),
      error_status_(false),
      stop_flag_(stop_flag),
      actuator_status_(false),
      sub_started_(false),
      stop_publisher_(false),
      nh_(ros::NodeHandle()),
      spinner_(2)
{
    actuator_ = std::make_shared<Linear_Actuator_Properties>(controller_name_ + "_actuator");
    input_.position_max = max_position;
    input_.speed_max = max_speed;
    input_.acceleration_max = max_acceleration;

    actuator_sub_ = nh_.subscribe(feedback_topic_, 1, &Linear_Actuator_Controller::_actuatorMsgCB, this);

    actuator_pub_ = nh_.advertise<linear_actuator_controller::LinearActuatorInput>(command_topic_, 1);

    spinner_.start();

    last_cb_ = ros::Time::now();
    publish_thread_ = std::thread(&Linear_Actuator_Controller::_publishCommand, this);
}

Linear_Actuator_Controller::~Linear_Actuator_Controller()
{
    try
    {
        disableActuators();
        stop_publisher_ = true;
        spinner_.stop();
        actuator_sub_.shutdown();
        actuator_pub_.shutdown();
    }
    catch (...)
    {
        ROS_ERROR("%s: unexpected error in desctructor", controller_name_.c_str());
    }
}

void Linear_Actuator_Controller::setActuator(Actuator_Properties_Ptr actuator)
{
    actuator_ = actuator;
}

void Linear_Actuator_Controller::readState()
{
    if (!_checkConnection())
    {
        return;
    }
    actuator_status_ = feedback_.active;
    actuator_->state.position = feedback_.position;
    actuator_->state.velocity = feedback_.speed;
    actuator_->state.effort = 0.0;
}

void Linear_Actuator_Controller::writeCommand()
{
    if (!actuator_status_)
    {
        stop_flag_ = true;
        error_status_ = true;
        error_msg_ = error_msg_ + " " + "not ready!";
    }
    else if (!_checkConnection())
    {
        return;
    }
    else
    {
        if (input_.position > actuator_->command.position &&
            actuator_->command.velocity > 0)
        {
            input_.speed = -actuator_->command.velocity;
        }
        else
        {
            input_.speed = actuator_->command.velocity;
        }
        input_.position = actuator_->command.position;
    }
}

void Linear_Actuator_Controller::enableActuators()
{
    input_.enabled = true;
    int wait_count = 0;
    while (wait_count < 5 && !actuator_status_)
    {
        ros::Duration(1 / publish_hz_).sleep();
        wait_count++;
    }
    if (!actuator_status_)
    {
        std::string error_str = actuator_->actuator_name + ": unexpected error, enable failed";
        ROS_ERROR(error_str.c_str());
        throw std::runtime_error("");
    }
}

void Linear_Actuator_Controller::disableActuators()
{
    input_.enabled = false;
    int wait_count = 0;
    while (wait_count < 5 && actuator_status_)
    {
        ros::Duration(1 / publish_hz_).sleep();
        wait_count++;
    }

    if (actuator_status_)
    {
        ROS_ERROR("%s: unexpected error, disable failed", actuator_->actuator_name.c_str());
    }
}

bool Linear_Actuator_Controller::getErrorDetails(std::string &error_msg)
{
    if (error_status_)
    {
        error_msg = (error_msg.empty() ? "" : (error_msg + "\n")) + actuator_->actuator_name + ":" + error_msg_;
    }
    return error_status_;
}

::Actuator_Properties_Ptr Linear_Actuator_Controller::getActuator(const std::string &name)
{
    return actuator_;
}

void Linear_Actuator_Controller::getActuatorNames(std::vector<std::string> &names)
{
    names.push_back(actuator_->actuator_name);
}

Linear_Actuator::Actuator_Properties_Ptr Linear_Actuator_Controller::getActuator()
{
    return actuator_;
}

void Linear_Actuator_Controller::_actuatorMsgCB(const linear_actuator_controller::LinearActuatorFeedback &feedback)
{
    sub_started_ = true;
    last_cb_ = ros::Time::now();
    feedback_ = feedback;
}

void Linear_Actuator_Controller::_publishCommand()
{
    ros::Rate rate(publish_hz_);
    while (!stop_publisher_)
    {
        actuator_pub_.publish(input_);
        rate.sleep();
    }
}

bool Linear_Actuator_Controller::_checkConnection()
{
    if (!sub_started_)
    {
        if ((ros::Time::now() - last_cb_).toSec() < 0.1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    if ((ros::Time::now() - last_cb_).toSec() > (10 / publish_hz_))
    {
        stop_flag_ = true;
        error_status_ = true;
        error_msg_ = error_msg_ + " " + "connection failed!";
        return false;
    }
    else
    {
        return true;
    }
}
