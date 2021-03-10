#ifndef PROPERTYVALUETABLE_H
#define PROPERTYVALUETABLE_H

#include <cstdint>
#include <vector>
#include "CRSDK/CameraRemote_SDK.h"
#include "Text.h"

namespace cli
{
template <typename T>
struct PropertyValueEntry
{
    bool writable;
    T current;
    std::vector<T> possible;
};

struct PropertyValueTable
{
    PropertyValueEntry<std::uint16_t> f_number;
    PropertyValueEntry<std::uint32_t> iso_sensitivity;
    PropertyValueEntry<std::uint32_t> shutter_speed;
    PropertyValueEntry<std::uint16_t> position_key_setting;
    PropertyValueEntry<std::uint32_t> exposure_program_mode;
    PropertyValueEntry<std::uint32_t> still_capture_mode;
    PropertyValueEntry<std::uint16_t> focus_mode;
    PropertyValueEntry<std::uint16_t> live_view_image_quality;
    PropertyValueEntry<std::uint16_t> live_view_status;
    PropertyValueEntry<std::uint8_t> media_slot1_format_enable_status;
    PropertyValueEntry<std::uint8_t> media_slot2_format_enable_status;
    PropertyValueEntry<std::uint16_t> white_balance;
    PropertyValueEntry<std::uint16_t> customwb_capture_stanby;
    PropertyValueEntry<std::uint16_t> customwb_capture_stanby_cancel;
    PropertyValueEntry<std::uint16_t> customwb_capture_operation;
    PropertyValueEntry<std::uint16_t> customwb_capture_execution_state;
    PropertyValueEntry<std::uint8_t> zoom_operation_status;
    PropertyValueEntry<std::uint8_t> zoom_setting_type;
    PropertyValueEntry<std::uint8_t> zoom_types_status;
    PropertyValueEntry<std::uint8_t> zoom_operation;
};

std::vector<std::uint16_t> parse_f_number(unsigned char const* buf, std::uint32_t nval);
std::vector<std::uint32_t> parse_iso_sensitivity(unsigned char const* buf, std::uint32_t nval);
std::vector<std::uint32_t> parse_shutter_speed(unsigned char const* buf, std::uint32_t nval);
std::vector<std::uint16_t> parse_position_key_setting(unsigned char const * buf, std::uint32_t nval);
std::vector<std::uint32_t> parse_exposure_program_mode(unsigned char const * buf, std::uint32_t nval);
std::vector<std::uint32_t> parse_still_capture_mode(unsigned char const * buf, std::uint32_t nval);
std::vector<std::uint16_t> parse_focus_mode(unsigned char const * buf, std::uint32_t nval);
std::vector<std::uint16_t> parse_live_view_image_quality(unsigned char const * buf, std::uint32_t nval);
std::vector<std::uint8_t> parse_media_slot1_format_enable_status(unsigned char const* buf, std::uint8_t nval);
std::vector<std::uint8_t> parse_media_slot2_format_enable_status(unsigned char const* buf, std::uint8_t nval);
std::vector<std::uint16_t> parse_white_balance(unsigned char const* buf, std::uint16_t nval);
std::vector<std::uint16_t> parse_customwb_capture_stanby(unsigned char const* buf, std::uint16_t nval);
std::vector<std::uint16_t> parse_customwb_capture_stanby_cancel(unsigned char const* buf, std::uint16_t nval);
std::vector<std::uint16_t> parse_customwb_capture_operation(unsigned char const* buf, std::uint16_t nval);
std::vector<std::uint16_t> parse_customwb_capture_execution_state(unsigned char const* buf, std::uint16_t nval);
std::vector<std::uint8_t> parse_zoom_operation_status(unsigned char const* buf, std::uint8_t nval);
std::vector<std::uint8_t> parse_zoom_setting_type(unsigned char const* buf, std::uint8_t nval);
std::vector<std::uint8_t> parse_zoom_types_status(unsigned char const* buf, std::uint8_t nval);
std::vector<std::uint8_t> parse_zoom_operation(unsigned char const* buf, std::uint8_t nval);

text format_f_number(std::uint16_t f_number);
text format_iso_sensitivity(std::uint32_t iso);
text format_shutter_speed(std::uint32_t shutter_speed);
text format_position_key_setting(std::uint16_t position_key_setting);
text format_exposure_program_mode(std::uint32_t exposure_program_mode);
text format_still_capture_mode(std::uint32_t still_capture_mode);
text format_focus_mode(std::uint16_t focus_mode);
text format_live_view_image_quality(std::uint16_t focus_mode);
text format_live_view_status(std::uint16_t focus_mode);
text format_media_slot1_format_enable_status(std::uint8_t media_slot1_format_enable_status);
text format_media_slot2_format_enable_status(std::uint8_t media_slot2_format_enable_status);
text format_white_balance(std::uint16_t value);
text format_customwb_capture_stanby(std::uint16_t customwb_capture_stanby);
text format_customwb_capture_stanby_cancel(std::uint16_t customwb_capture_stanby_cancel);
text format_customwb_capture_operation(std::uint16_t customwb_capture_operation);
text format_customwb_capture_execution_state(std::uint16_t customwb_capture_execution_state);
text format_zoom_operation_status(std::uint8_t zoom_operation_status);
text format_zoom_setting_type(std::uint8_t zoom_setting_type);
text format_zoom_types_status(std::uint8_t zoom_types_status);
text format_zoom_operation(std::uint8_t zoom_operation);
} // namespace cli

#endif // !PROPERTYVALUETABLE_H
