#ifndef __LINEAR_ACTUATOR_TRANSMISSION_LOADER_H__
#define __LINEAR_ACTUATOR_TRANSMISSION_LOADER_H__

#include <configurable_control_hw/Transmission_Loader_Plugin.h>

namespace Linear_Actuator
{
    class Transmission_Loader : public Transmission_Loader_Plugin
    {
        public:
        virtual Transmission_Ptr loadTransmission(XmlRpc::XmlRpcValue& config) override;
    };
}

#endif