#include <pluginlib/class_list_macros.h>


#include <transmission_interface/simple_transmission.h>
#include "Linear_Actuator_Transmission_Loader.h"

using namespace Linear_Actuator;

Transmission_Ptr Transmission_Loader::loadTransmission(XmlRpc::XmlRpcValue& config)
{
    double min_step_deg = config["min_step_deg"];
    int micro_steps = config["micro_steps"];
    double lead = config["lead"];

    double reduction = (360*micro_steps) /(lead * min_step_deg);

    return std::make_shared<transmission_interface::SimpleTransmission>(reduction);
}

PLUGINLIB_EXPORT_CLASS(Linear_Actuator::Transmission_Loader, Transmission_Loader_Plugin)