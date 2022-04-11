#ifndef __LINEAR_ACTUATOR_PROPERTIES_H__
#define __LINEAR_ACTUATOR_PROPERTIES_H__

#include <configurable_control_hw/Actuator_Properties.h>

#include <memory>
#include <string>

namespace Linear_Actuator
{

const std::string ACTUATOR_NAME_STR = "linear_actuator";

struct Linear_Actuator_Properties : public Actuator_Properties
{
    Linear_Actuator_Properties(const std::string& name)
    {
        actuator_name =  name;
        actuator_type = ACTUATOR_NAME_STR;
    }
};

typedef std::shared_ptr<Linear_Actuator_Properties> Actuator_Properties_Ptr;

}

#endif