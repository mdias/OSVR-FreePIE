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

#pragma once

#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

#include <memory>
#include <boost/asio.hpp>

namespace FreePieOsvr {
    struct DeviceConfig
    {
        unsigned char channel;
        std::string name;
        unsigned short port;
    };

    class Device {
        public:
            Device(OSVR_PluginRegContext ctx, const DeviceConfig& cfg);
            ~Device();

            OSVR_ReturnCode update();

        private:
            void m_fConnectSocket( unsigned short port );
            void m_fDisconnectSocket();
            bool m_fReceiveData( OSVR_PoseState& out, std::size_t maxWaitMilisecs );

            osvr::pluginkit::DeviceToken m_dev;
            OSVR_TrackerDeviceInterface m_tracker;

            DeviceConfig m_cfg;

            boost::asio::io_service m_io_service;
            std::unique_ptr<boost::asio::ip::udp::socket> m_udp_socket;
            boost::asio::ip::udp::endpoint m_udp_local_endpoint;
            boost::asio::ip::udp::endpoint m_udp_remote_endpoint;
    };
}
