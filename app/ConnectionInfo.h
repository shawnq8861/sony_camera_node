#ifndef DEVICEID_H
#define DEVICEID_H

#include "Text.h"

namespace cli
{

enum class ConnectionType
{
    UNKNOWN,
    NETWORK,
    USB
};

struct NetworkInfo
{
    std::uint32_t ip_address;
    text mac_address;
};

struct UsbInfo
{
    std::int16_t pid;
};

ConnectionType parse_connection_type(text conn_type);
NetworkInfo parse_ip_info(unsigned char const* buf, std::uint32_t size);

} // namespace cli

#endif // !DEVICEID_H
