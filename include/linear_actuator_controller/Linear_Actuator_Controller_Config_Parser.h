#ifndef __LINEAR_ACTUATOR_CONTROLLER_CONFIG_PARSER_H__
#define __LINEAR_ACTUATOR_CONTROLLER_CONFIG_PARSER_H__

#include <configurable_control_hw/Actuator_Config_Parser.h>

namespace Linear_Actuator
{
class Linear_Actuator_Controller_Config_Parser : public Actuator_Config_Parser
{
public:
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map) override;
};
}

#endif