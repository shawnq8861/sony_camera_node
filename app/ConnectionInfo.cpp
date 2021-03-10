#include "ConnectionInfo.h"
#include <cstddef>
#include <cstring>
#include "CRSDK/CameraRemote_SDK.h"

namespace impl
{
constexpr std::size_t const MACADDRESSBYTE    = 6;
constexpr std::size_t const CAMERANAMELENGMAX = 256;
constexpr std::size_t const CAMERADESCLENGMAX = 256;

struct NetworkDeviceInfo
{
    CrInt32u idsize;
    CrInt32u ipaddress;
    CrInt8u  name[CAMERANAMELENGMAX];
    CrInt8u  desc[CAMERADESCLENGMAX];
    CrInt8u  MACaddress[MACADDRESSBYTE];
    CrInt32u urlsize;
};

} // namespace impl

namespace cli
{
ConnectionType parse_connection_type(text conn_type)
{
    ConnectionType type(ConnectionType::UNKNOWN);

    if (conn_type == TEXT("PTP-IP")) {
        type = ConnectionType::NETWORK;
    }
    else if (conn_type == TEXT("USB")) {
        type = ConnectionType::USB;
    }
    return type;
}

NetworkInfo parse_ip_info(unsigned char const* buf, std::uint32_t size)
{
    NetworkInfo info;
    impl::NetworkDeviceInfo raw_info;
    std::memcpy(&raw_info, buf, sizeof raw_info);

    info.ip_address = raw_info.ipaddress;
    info.mac_address = text(reinterpret_cast<text_literal>(raw_info.MACaddress));
    return info;
}

} // namespace cli
