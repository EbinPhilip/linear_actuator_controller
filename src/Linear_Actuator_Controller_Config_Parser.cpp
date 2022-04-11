#include <pluginlib/class_list_macros.h>

#include "Linear_Actuator_Controller_Config_Parser.h"
#include <configurable_control_hw/Actuator_Controller.h>
#include "Linear_Actuator_Controller.h"

#include <memory>
#include <stdexcept>
#include <algorithm>

using namespace XmlRpc;
using namespace Linear_Actuator;

void Linear_Actuator_Controller_Config_Parser::parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map)
{
    if (config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("controller config parsing failed!");
    }
    
    for(auto it = config.begin(); it != config.end(); ++it)
    {
        if (it->second.getType() != XmlRpcValue::Type::TypeStruct)
        {
            throw std::runtime_error("actuator config parsing failed!");
        }
    
        std::string controller_name = it->first;
        std::string command_topic = it->second["command_topic"];
        std::string feedback_topic = it->second["feedback_topic"];
        double publish_hz = it->second["publish_hz"];
        int max_position = it->second["max_position"];
        float max_speed = (float)static_cast<double>(it->second["max_speed"]);
        float max_acceleration = (float)static_cast<double>(it->second["max_acceleration"]);
      

        std::shared_ptr<Linear_Actuator_Controller> controller_ptr = std::make_shared<Linear_Actuator_Controller>
            (controller_name, command_topic, feedback_topic, publish_hz, max_position, max_speed, max_acceleration, *stop_flag_ptr_);
      
        controller_map->insert(std::make_pair(controller_name, controller_ptr));
    }
}

PLUGINLIB_EXPORT_CLASS(Linear_Actuator::Linear_Actuator_Controller_Config_Parser, Actuator_Config_Parser)