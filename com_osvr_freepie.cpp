// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>

// Generated JSON header file
#include "com_osvr_freepie_json.h"

// Library/third-party includes
#include <json/value.h>
#include <json/reader.h>

// Standard includes
#include <iostream>
#include <memory>

//
#include "device.h"

// Anonymous namespace to avoid symbol collision
namespace {
    class ConfiguredDeviceConstructor {
        public:
            OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char *params) {
                // Read the JSON data from parameters.
                Json::Value root;
                if( params ) {
                    Json::Reader r;
                    if( !r.parse(params, root) ) {
                        std::cerr << "Could not parse parameters!" << std::endl;
                    }
                }

                // read configs
                FreePieOsvr::DeviceConfig cfg;
                cfg.channel = 0;
                cfg.port = 5555;

                if( root.isMember("channel") ) {
                    int i = root.get("channel", 0).asInt();
                    if( i>=0 && i<=15 ) {
                        cfg.port = (unsigned short)i;
                    } else {
                        std::cerr << "Invalid channel specified. Valid range is 0-15.";
                    }
                }

                if( root.isMember("port") ) {
                    int i = root.get("port", 5555).asInt();
                    if( i>=0 && i<=65535 ) {
                        cfg.port = (unsigned short)i;
                    } else {
                        std::cerr << "Invalid port specified";
                    }
                }

                if( root.isMember("name") ) {
                    cfg.name = root.get("name", "").asString();
                }


                // create device
                m_devices.emplace_back( new FreePieOsvr::Device(ctx, cfg) );

                return OSVR_RETURN_SUCCESS;
            }

        protected:
             std::vector< std::unique_ptr<FreePieOsvr::Device> > m_devices;
    };

} // namespace

OSVR_PLUGIN(com_osvr_freepie) {
    osvr::pluginkit::PluginContext context(ctx);

    /// Tell the core we're available to create a device object.
    osvr::pluginkit::registerDriverInstantiationCallback(
        ctx,
        "FreePIE",
        new ConfiguredDeviceConstructor
    );

    return OSVR_RETURN_SUCCESS;
}
