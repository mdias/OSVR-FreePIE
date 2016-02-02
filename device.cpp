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

// Generated JSON header file
#include "com_osvr_freepie_json.h"

// Standard includes
#include <chrono>
#include <thread>
#include <iostream>
#include <ctype.h>
#include <math.h>

//
#include "device.h"

using namespace FreePieOsvr;

namespace
{
#pragma pack(push, 1)
    struct FREEPIE_DATA
    {
        enum F {
            FLAG_RAW = 1 << 0,
            FLAG_ORIENTATION = 1 << 1,
            FLAG_ALL = FLAG_RAW | FLAG_ORIENTATION
        };

        std::uint8_t channel;
        std::uint8_t flags;
        float data[12];

        bool getOrientation( float& yaw, float& pitch, float& roll )
        {
            if( flags == FLAG_ORIENTATION )
            {
                yaw = data[0];
                pitch = data[1];
                roll = data[2];
                return true;
            }
            else if( flags == (FLAG_ORIENTATION | FLAG_RAW) )
            {
                yaw = data[9];
                pitch = data[10];
                roll = data[11];
                return true;
            }

            return false;
        }
    };
#pragma pack(pop)

    static void convertEulerToQuaternion( OSVR_Quaternion& out, float yaw, float pitch, float roll )
    {
        float c1 = cosf(yaw / 2.f);
        float s1 = sinf(yaw / 2.f);
        float c2 = cosf(pitch / 2.f);
        float s2 = sinf(pitch / 2.f);
        float c3 = cosf(roll / 2.f);
        float s3 = sinf(roll / 2.f);
        float c1c2 = c1*c2;
        float s1s2 = s1*s2;
        out.data[0] = c1c2*c3 - s1s2*s3;
        out.data[1] = c1c2*s3 + s1s2*c3;
        out.data[2] = s1*c2*c3 + c1*s2*s3;
        out.data[3] = c1*s2*c3 - s1*c2*s3;
    }
}

///
/// \brief Device::Device
/// \param ctx
///
Device::Device(OSVR_PluginRegContext ctx, const DeviceConfig& cfg)
    : m_cfg( cfg )
{
    m_fConnectSocket( m_cfg.port );

    /// Create the initialization options
    OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

    // create tracker interface
    osvrDeviceTrackerConfigure(opts, &m_tracker);

    /// Create the async device token with the options
    m_dev.initAsync(ctx, m_cfg.name.length() ? m_cfg.name.c_str() : "FreePIE", opts);

    /// Send JSON descriptor
    m_dev.sendJsonDescriptor(com_osvr_freepie_json);

    /// Register update callback
    m_dev.registerUpdateCallback(this);
}

///
/// \brief Device::~Device
///
Device::~Device()
{
    m_fDisconnectSocket();
}

///
/// \brief Device::update
/// \return
///
OSVR_ReturnCode Device::update()
{
    OSVR_PoseState pose;
    osvrPose3SetIdentity( &pose );

    bool res = m_fReceiveData(pose, 150);

    if( !res ) {
        return OSVR_RETURN_FAILURE;
    }

    osvrDeviceTrackerSendPose(m_dev, m_tracker, &pose, 0);

    return OSVR_RETURN_SUCCESS;
}

///
/// \brief Device::m_fConnectSocket
/// \param port
///
void Device::m_fConnectSocket( unsigned short port )
{
    m_fDisconnectSocket();

    using namespace boost::asio::ip;

    // setup endpoints
    m_udp_local_endpoint = udp::endpoint( udp::v4(), port );
    m_udp_remote_endpoint = udp::endpoint( udp::v4(), 0 );

    // create socket
    m_udp_socket.reset( new boost::asio::ip::udp::socket(m_io_service) );

    // open the socket and bind it to the local endpoint
    m_udp_socket->open( udp::v4() );
    m_udp_socket->bind( m_udp_local_endpoint );
    m_io_service.run();

}

///
/// \brief Device::m_fDisconnectSocket
///
void Device::m_fDisconnectSocket()
{
    if( m_udp_socket )
    {
        m_udp_socket->cancel();
        m_udp_socket->close();
        m_udp_socket.reset();
    }

    m_io_service.stop();
}

///
/// \brief Device::m_fReceiveData
/// \param out
/// \param maxWaitMilisecs
/// \return
///
bool Device::m_fReceiveData( OSVR_PoseState& out, std::size_t maxWaitMilisecs )
{
    std::size_t waited = 0;
    boost::system::error_code recv_err;

    FREEPIE_DATA recv_buffer;
    bool hasData = false;

    while( waited <= maxWaitMilisecs )
    {
        for( ;; )
        {
            // check for bytes available
            auto numBytesAvailable = m_udp_socket->available();

            if( numBytesAvailable > 0 ) {
                //std::cout << "Num bytes: " << numBytesAvailable << std::endl;
            }

            // we need at least the flags and 3 floats
            if( numBytesAvailable < (2 + 3*sizeof(float)) ) {
                break;
            }

            // receive the data
            auto bytes_received = m_udp_socket->receive_from(
                boost::asio::buffer( &recv_buffer, sizeof(recv_buffer) ),
                m_udp_remote_endpoint,
                0,
                recv_err
            );

            if( recv_err )
            {
                std::cerr << "Error receiving UDP data! " << recv_err.message() << std::endl;
                return false;
            }

            // check the channel
            if( recv_buffer.channel != m_cfg.channel ) {
                //std::cout << "channel: " << (int)recv_buffer.channel << std::endl;
                break;
            }

            // output the orientation data
            float yaw,pitch,roll;
            if( recv_buffer.getOrientation( yaw, pitch, roll ) )
            {
                convertEulerToQuaternion(
                    out.rotation,
                    yaw, pitch, roll
                );
                hasData = true;

                //std::cout << "yaw: " << yaw << "   pitch: " << pitch << "   roll: " << roll << std::endl;
            }
        }

        if( hasData ) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ++waited;
    }

    return false;
}
