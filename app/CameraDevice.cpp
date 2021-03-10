#include "CameraDevice.h"
#include <chrono>
#if defined(__GNUC__) && __GNUC__ < 8
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#include <fstream>
#include <thread>
#include "CRSDK/CrDeviceProperty.h"
#include "Text.h"

namespace SDK = SCRSDK;
using namespace std::chrono_literals;

//constexpr int const ImageSaveAutoStartNo = -1;
constexpr int const ImageSaveAutoStartNo = 100;

namespace cli
{
CameraDevice::CameraDevice(CRLibInterface const* cr_lib, SCRSDK::ICrCameraObjectInfo const* camera_info)
    : m_cr_lib(cr_lib)
    , m_device_handle(0)
    , m_connected(false)
    , m_conn_type(ConnectionType::UNKNOWN)
    , m_net_info()
    , m_usb_info()
    , m_prop()
{
    m_info = SDK::CreateCameraObjectInfo(
        camera_info->GetName(),
        camera_info->GetModel(),
        camera_info->GetUsbPid(),
        camera_info->GetIdType(),
        camera_info->GetIdSize(),
        camera_info->GetId(),
        camera_info->GetConnectionTypeName(),
        camera_info->GetAdaptorName(),
        camera_info->GetPairingNecessity()
    );

    m_conn_type = parse_connection_type(m_info->GetConnectionTypeName());
    switch (m_conn_type)
    {
    case ConnectionType::NETWORK:
        m_net_info = parse_ip_info(m_info->GetId(), m_info->GetIdSize());
        break;
    case ConnectionType::USB:
        m_usb_info.pid = m_info->GetUsbPid();
        break;
    case ConnectionType::UNKNOWN:
        [[fallthrough]];
    default:
        // Do nothing
        break;
    }
}

CameraDevice::~CameraDevice()
{
    if (m_info) m_info->Release();
}

bool CameraDevice::connect()
{
    // auto connect_status = m_cr_lib->Connect(m_info, this, &m_device_handle);
    auto connect_status = SDK::Connect(m_info, this, &m_device_handle);
    if (CR_FAILED(connect_status)) {
        return false;
    }
    set_save_info();
    return true;
}

bool CameraDevice::disconnect()
{
    tout << "Disconnect from camera...\n";
    // auto disconnect_status = m_cr_lib->Disconnect(m_device_handle);
    auto disconnect_status = SDK::Disconnect(m_device_handle);
    if (CR_FAILED(disconnect_status)) {
        tout << "Disconnect failed to initialize.\n";
        return false;
    }

    tout << "Finalize camera...\n";
    // auto finalize_status = m_cr_lib->FinalizeDevice(m_device_handle);
    auto finalize_status = SDK::ReleaseDevice(m_device_handle);
    if (CR_FAILED(finalize_status)) {
        tout << "Finalize device failed to initialize.\n";
        return false;
    }

    return true;
}

void CameraDevice::capture_image() const
{
    tout << "Capture image...\n";
    tout << "Shutter down\n";
    // m_cr_lib->SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_Release, SDK::CrCommandParam_Down);
    SDK::SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_Release, SDK::CrCommandParam_Down);

    // Wait, then send shutter up
    std::this_thread::sleep_for(35ms);
    tout << "Shutter up\n";
    // m_cr_lib->SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_Release, SDK::CrCommandParam_Up);
    SDK::SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_Release, SDK::CrCommandParam_Up);
}

void CameraDevice::s1_shooting() const
{
    text input;
    tout << "Is the focus mode set to AF? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Set the focus mode to AF\n";
        return;
    }

    tout << "S1 shooting...\n";
    tout << "Shutter Halfpress down\n";
    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_S1);
    prop.SetCurrentValue(SDK::CrLockIndicator::CrLockIndicator_Locked);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16);
    SDK::SetDeviceProperty(m_device_handle, &prop);

    // Wait, then send shutter up
    std::this_thread::sleep_for(1s);
    tout << "Shutter Halfpress up\n";
    prop.SetCurrentValue(SDK::CrLockIndicator::CrLockIndicator_Unlocked);
    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::af_shutter() const
{
    text input;
    tout << "Is the focus mode set to AF? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Set the focus mode to AF\n";
        return;
    }

    tout << "S1 shooting...\n";
    tout << "Shutter Halfpress down\n";
    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_S1);
    prop.SetCurrentValue(SDK::CrLockIndicator::CrLockIndicator_Locked);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16);
    SDK::SetDeviceProperty(m_device_handle, &prop);

    // Wait, then send shutter down
    std::this_thread::sleep_for(500ms);
    tout << "Shutter down\n";
    SDK::SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_Release, SDK::CrCommandParam::CrCommandParam_Down);

    // Wait, then send shutter up
    std::this_thread::sleep_for(35ms);
    tout << "Shutter up\n";
    SDK::SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_Release, SDK::CrCommandParam::CrCommandParam_Up);

    // Wait, then send shutter up
    std::this_thread::sleep_for(1s);
    tout << "Shutter Halfpress up\n";
    prop.SetCurrentValue(SDK::CrLockIndicator::CrLockIndicator_Unlocked);
    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::continuous_shooting() const
{
    tout << "Capture image...\n";
    tout << "Continuous Shooting\n";

    // Set, PriorityKeySettings property
    SDK::CrDeviceProperty priority;
    priority.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_PriorityKeySettings);
    priority.SetCurrentValue(SDK::CrPriorityKeySettings::CrPriorityKey_PCRemote);
    priority.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);
    auto err_priority = SDK::SetDeviceProperty(m_device_handle, &priority);
    if (CR_FAILED(err_priority)) {
        tout << "Priority Key setting FAILED\n";
        return;
    }
    else {
        tout << "Priority Key setting SUCCESS\n";
    }

    // Set, still_capture_mode property
    SDK::CrDeviceProperty mode;
    mode.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_DriveMode);
    mode.SetCurrentValue(SDK::CrDriveMode::CrDrive_Continuous_Hi);
    mode.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);
    auto err_still_capture_mode = SDK::SetDeviceProperty(m_device_handle, &mode);
    if (CR_FAILED(err_still_capture_mode)) {
        tout << "Still Capture Mode setting FAILED\n";
        return;
    }
    else {
        tout << "Still Capture Mode setting SUCCESS\n";
    }

    // get_still_capture_mode();
    std::this_thread::sleep_for(1s);
    tout << "Shutter down\n";
    SDK::SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_Release, SDK::CrCommandParam::CrCommandParam_Down);

    // Wait, then send shutter up
    std::this_thread::sleep_for(500ms);
    tout << "Shutter up\n";
    SDK::SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_Release, SDK::CrCommandParam::CrCommandParam_Up);
}

void CameraDevice::get_aperture()
{
    load_properties();
    tout << format_f_number(m_prop.f_number.current) << '\n';
}

void CameraDevice::get_iso()
{
    load_properties();

    std::uint32_t iso_mode = (m_prop.iso_sensitivity.current >> 28);
    if (iso_mode == 0x0000) {
        // Normal mode
        tout << "ISO Mode: Normal\n";
    }
    else if (iso_mode == 0x0001) {
        // Multi Frame mode
        tout << "ISO Mode: Multi Frame\n";
    }
    else if (iso_mode == 0x0002) {
        // Multi Frame High mode
        tout << "ISO Mode: Multi Frame High\n";
    }

    tout << "ISO " << format_iso_sensitivity(m_prop.iso_sensitivity.current) << '\n';
}

void CameraDevice::get_shutter_speed()
{
    load_properties();
    tout << "Shutter speed: " << format_shutter_speed(m_prop.shutter_speed.current) << '\n';
}

void CameraDevice::get_position_key_setting()
{
    load_properties();
    tout << "Position Key Setting: " << format_position_key_setting(m_prop.position_key_setting.current) << '\n';
}

void CameraDevice::get_exposure_program_mode()
{
    load_properties();
    tout << "Exposure Program Mode: " << format_exposure_program_mode(m_prop.exposure_program_mode.current) << '\n';
}

void CameraDevice::get_still_capture_mode()
{
    load_properties();
    tout << "Still Capture Mode: " << format_still_capture_mode(m_prop.still_capture_mode.current) << '\n';
}

void CameraDevice::get_focus_mode()
{
    load_properties();
    tout << "Focus Mode: " << format_focus_mode(m_prop.focus_mode.current) << '\n';
}

void CameraDevice::get_live_view()
{
    tout << "GetLiveView...\n";

    CrInt32 num = 0;
    SDK::CrLiveViewProperty* property = NULL;
    auto err = SDK::GetLiveViewProperties(m_device_handle, &property, &num);
    if (CR_FAILED(err)) {
        tout << "GetLiveView FAILED\n";
        return;
    }

    SDK::CrImageInfo inf;
    err = SDK::GetLiveViewImageInfo(m_device_handle, &inf);
    if (CR_FAILED(err)) {
        tout << "GetLiveView FAILED\n";
        return;
    }

    CrInt32u bufSize = inf.GetBufferSize();
    tout << "buffer size " << bufSize << " \n";
    if (bufSize < 1)
    {
        tout << "GetLiveView FAILED \n";
    }
    else
    {
        auto* image_data = new SDK::CrImageDataBlock();
        if (!image_data)
        {
            tout << "GetLiveView FAILED (new CrImageDataBlock class)\n";
            return;
        }
        CrInt8u* image_buff = new CrInt8u[bufSize];
        if (!image_buff)
        {
            delete image_data;
            tout << "GetLiveView FAILED (new Image buffer)\n";
            return;
        }
        image_data->SetSize(bufSize);
        image_data->SetData(image_buff);

        err = SDK::GetLiveViewImage(m_device_handle, image_data);

        if (CR_FAILED(err))
        {
            // FAILED
            if (err == SDK::CrWarning_Frame_NotUpdated) {
                tout << "Warning. GetLiveView Frame NotUpdate\n";
            }
            else if (err == SDK::CrError_Memory_Insufficient) {
                tout << "Warning. GetLiveView Memory insufficient\n";
            }
            delete[] image_buff; // Release
            delete image_data; // Release
        }
        else
        {
            if (0 < image_data->GetSize())
            {
                // Display
                // etc.
                auto path = fs::current_path();
                path.append(TEXT("LiveView000000.JPG"));
                tout << path << '\n';

                std::ofstream file(path, std::ios::out | std::ios::binary);
                if (!file.bad())
                {
                    file.write((char*)image_data->GetImageData(), image_data->GetImageSize());
                    file.close();
                }
                tout << "GetLiveView SUCCESS\n";
                delete[] image_buff; // Release
                delete image_data; // Release
            }
            else
            {
                // FAILED
                delete[] image_buff; // Release
                delete image_data; // Release
            }
        }
    }
}

void CameraDevice::get_live_view_image_quality()
{
    load_properties();
    tout << "Live View Imege Quality: " << format_live_view_image_quality(m_prop.live_view_image_quality.current) << '\n';
}

void CameraDevice::get_live_view_status()
{
    load_properties();
    tout << "LiveView Enabled: " << format_live_view_status(m_prop.live_view_status.current) << '\n';
}

void CameraDevice::get_select_media_format()
{
    load_properties();
    tout << "Media SLOT1 Format Enable Status: " << format_media_slot1_format_enable_status(m_prop.media_slot1_format_enable_status.current) << '\n';
    tout << "Media SLOT2 Format Enable Status: " << format_media_slot2_format_enable_status(m_prop.media_slot2_format_enable_status.current) << '\n';
}

void CameraDevice::get_white_balance()
{
    load_properties();
    tout << "White Balance: " << format_white_balance(m_prop.white_balance.current) << '\n';
}

bool CameraDevice::get_custom_wb()
{
    bool state = false;
    load_properties();
    tout << "CustomWB Capture Standby Operation: " << format_customwb_capture_stanby(m_prop.customwb_capture_stanby.current) << '\n';
    tout << "CustomWB Capture Standby CancelOperation: " << format_customwb_capture_stanby_cancel(m_prop.customwb_capture_stanby_cancel.current) << '\n';
    tout << "CustomWB Capture Operation: " << format_customwb_capture_operation(m_prop.customwb_capture_operation.current) << '\n';
    tout << "CustomWB Capture Execution State : " << format_customwb_capture_execution_state(m_prop.customwb_capture_execution_state.current) << '\n';
    if (m_prop.customwb_capture_operation.current == 1) {
        state = true;
    }
    return state;
}

void CameraDevice::get_zoom_operation()
{
    load_properties();
    tout << "Zoom Operation Status: " << format_zoom_operation_status(m_prop.zoom_operation_status.current) << '\n';
    tout << "Zoom Setting Type: " << format_zoom_setting_type(m_prop.zoom_setting_type.current) << '\n';
    tout << "Zoom Type Status: " << format_zoom_types_status(m_prop.zoom_types_status.current) << '\n';
    tout << "Zoom Operation: " << format_zoom_operation(m_prop.zoom_operation.current) << '\n';
   
    std::int32_t nprop = 0;
    SDK::CrDeviceProperty* prop_list = nullptr;
    auto status = SDK::GetDeviceProperties(m_device_handle, &prop_list, &nprop);

    if (CR_FAILED(status)) {
        tout << "Failed to get device properties.\n";
        return;
    }

    if (prop_list && nprop > 0) {
        // Got properties list
        for (std::int32_t i = 0; i < nprop; ++i) {
            auto prop = prop_list[i];

            switch (prop.GetCode())
            {
            case SDK::CrDevicePropertyCode::CrDeviceProperty_Zoom_Bar_Information: // get only
                tout << "Zoom Bar Information: 0x" << std::hex << prop.GetCurrentValue() << std::dec << '\n';
                break;
            }
        }
    }
}

bool CameraDevice::get_image_data(SCRSDK::CrImageDataBlock *image_data, CrInt8u *image_buff)
{
    tout << "Get Image data...\n";

    SDK::CrImageInfo inf;
    auto err = SDK::GetLiveViewImageInfo(m_device_handle, &inf);
    if (CR_FAILED(err)) {
        tout << "Get Image data FAILED get info err\n";
        return false;
    }
    CrInt32u bufSize = inf.GetBufferSize();
    tout << "bufSize = " << bufSize << "\n";
    if (bufSize < 1) {
        tout << "Get Image Data FAILED buffer size\n";
        return false;
    }
    else {
        if (!image_data) {
            tout << "Get Image Data FAILED (new CrImageDataBlock class)\n";
            return false;
        }
        image_buff = new CrInt8u[bufSize];
        if (!image_buff) {
            tout << "Get Image data FAILED (new Image buffer)\n";
            return false;
        }
        image_data->SetSize(bufSize);
        image_data->SetData(image_buff);

        err = SDK::GetLiveViewImage(m_device_handle, image_data);
        if (CR_FAILED(err)) {
            // FAILED
            if (err == SDK::CrWarning_Frame_NotUpdated) {
                tout << "Warning. Get Image Data Frame Not Update\n";
            }
            else if (err == SDK::CrError_Memory_Insufficient) {
                tout << "Warning. Get Image Data Memory insufficient\n";
            }
            return false;
        }
        else {
            if (0 < image_data->GetSize()) {
                tout << "Get Image Data SUCCESS, image size = " 
                     << image_data->GetSize() << "\n";
            }
            else {
                // FAILED
                return false;
            }
        }
    }
    return true;
}

void CameraDevice::set_aperture()
{
    if (!m_prop.f_number.writable) {
        // Not a settable property
        tout << "Aperture is not writable\n";
        return;
    }

    text input;
    tout << "Would you like to set a new Aperture value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << "Choose a number set a new Aperture value:\n";
    tout << "[-1] Cancel input\n";

    auto& values = m_prop.f_number.possible;
    for (std::size_t i = 0; i < values.size(); ++i) {
        tout << '[' << i << "] " << format_f_number(values[i]) << '\n';
    }

    tout << "[-1] Cancel input\n";
    tout << "Choose a number set a new Aperture value:\n";

    tout << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || values.size() <= selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FNumber);
    prop.SetCurrentValue(values[selected_index]);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

    // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::set_iso()
{
    if (!m_prop.iso_sensitivity.writable) {
        // Not a settable property
        tout << "ISO is not writable\n";
        return;
    }

    text input;
    tout << "Would you like to set a new ISO value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << "Choose a number set a new ISO value:\n";
    tout << "[-1] Cancel input\n";

    auto& values = m_prop.iso_sensitivity.possible;
    for (std::size_t i = 0; i < values.size(); ++i) {
        tout << '[' << i << "] ISO " << format_iso_sensitivity(values[i]) << '\n';
    }

    tout << "[-1] Cancel input\n";
    tout << "Choose a number set a new ISO value:\n";

    tout << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || values.size() <= selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_IsoSensitivity);
    prop.SetCurrentValue(values[selected_index]);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

    // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
    SDK::SetDeviceProperty(m_device_handle, &prop);
}

bool CameraDevice::set_save_info() const
{
    text path = fs::current_path().native();
    tout << path.data() << '\n';
    CrChar *prefix = TEXT("AAA");
    tout << "prefix = " << prefix << "\n";
    CrInt32 no = 100;
    tout << "no = " << no << "\n";

    // auto save_status = m_cr_lib->SetSaveInfo(m_device_handle
    //     , const_cast<text_char*>(path.data()), TEXT("DCS"), 1);
    //auto save_status = SDK::SetSaveInfo(m_device_handle
    //    , const_cast<text_char*>(path.data()), TEXT("AAA"), ImageSaveAutoStartNo);
    auto save_status = SDK::SetSaveInfo(m_device_handle
        , const_cast<text_char*>(path.data()), prefix, no);
    if (CR_FAILED(save_status)) {
        tout << "Failed to set save path.\n";
        return false;
    }
    return true;
}

void CameraDevice::set_shutter_speed()
{
    if (!m_prop.shutter_speed.writable) {
        // Not a settable property
        tout << "Shutter Speed is not writable\n";
        return;
    }

    text input;
    tout << "Would you like to set a new Shutter Speed value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << "Choose a number set a new Shutter Speed value:\n";
    tout << "[-1] Cancel input\n";

    auto& values = m_prop.shutter_speed.possible;
    for (std::size_t i = 0; i < values.size(); ++i) {
        tout << '[' << i << "] " << format_shutter_speed(values[i]) << '\n';
    }

    tout << "[-1] Cancel input\n";
    tout << "Choose a number set a new Shutter Speed value:\n";

    tout << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || values.size() <= selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_ShutterSpeed);
    prop.SetCurrentValue(values[selected_index]);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

    // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::set_position_key_setting()
{
    if (!m_prop.position_key_setting.writable) {
        // Not a settable property
        tout << "Position Key Setting is not writable\n";
        return;
    }

    text input;
    tout << "Would you like to set a new Position Key Setting value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << "Choose a number set a new Position Key Setting value:\n";
    tout << "[-1] Cancel input\n";

    auto& values = m_prop.position_key_setting.possible;
    for (std::size_t i = 0; i < values.size(); ++i) {
        tout << '[' << i << "] " << format_position_key_setting(values[i]) << '\n';
    }

    tout << "[-1] Cancel input\n";
    tout << "Choose a number set a new Position Key Setting value:\n";

    tout << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || values.size() <= selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_PriorityKeySettings);
    prop.SetCurrentValue(values[selected_index]);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt8Array);

    // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::set_exposure_program_mode()
{
    if (!m_prop.exposure_program_mode.writable) {
        // Not a settable property
        tout << "Exposure Program Mode is not writable\n";
        return;
    }

    text input;
    tout << "Would you like to set a new Exposure Program Mode value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << "Choose a number set a new Exposure Program Mode value:\n";
    tout << "[-1] Cancel input\n";

    auto& values = m_prop.exposure_program_mode.possible;
    for (std::size_t i = 0; i < values.size(); ++i) {
        tout << '[' << i << "] " << format_exposure_program_mode(values[i]) << '\n';
    }

    tout << "[-1] Cancel input\n";
    tout << "Choose a number set a new Exposure Program Mode value:\n";

    tout << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || values.size() <= selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_ExposureProgramMode);
    prop.SetCurrentValue(values[selected_index]);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

    // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::set_still_capture_mode()
{
    if (!m_prop.still_capture_mode.writable) {
        // Not a settable property
        tout << "Still Capture Mode is not writable\n";
        return;
    }

    text input;
    tout << "Would you like to set a new Still Capture Mode value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << "Choose a number set a new Still Capture Mode value:\n";
    tout << "[-1] Cancel input\n";

    auto& values = m_prop.still_capture_mode.possible;
    for (std::size_t i = 0; i < values.size(); ++i) {
        tout << '[' << i << "] " << format_still_capture_mode(values[i]) << '\n';
    }

    tout << "[-1] Cancel input\n";
    tout << "Choose a number set a new Still Capture Mode value:\n";

    tout << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || values.size() <= selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_DriveMode);
    prop.SetCurrentValue(values[selected_index]);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

    // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::set_focus_mode()
{
    if (!m_prop.focus_mode.writable) {
        // Not a settable property
        tout << "Focus Mode is not writable\n";
        return;
    }

    text input;
    tout << "Would you like to set a new Focus Mode value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << "Choose a number set a new Focus Mode value:\n";
    tout << "[-1] Cancel input\n";

    auto& values = m_prop.focus_mode.possible;
    for (std::size_t i = 0; i < values.size(); ++i) {
        tout << '[' << i << "] " << format_focus_mode(values[i]) << '\n';
    }

    tout << "[-1] Cancel input\n";
    tout << "Choose a number set a new Focus Mode value:\n";

    tout << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || values.size() <= selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FocusMode);
    prop.SetCurrentValue(values[selected_index]);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

    // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::set_live_view_image_quality()
{
    if (!m_prop.live_view_image_quality.writable) {
        // Not a settable property
        tout << "Live View Image Quality is not writable\n";
        return;
    }

    text input;
    tout << "Would you like to set a new Live View Image Quality value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << "Choose a number set a new Live View Image Quality value:\n";
    tout << "[-1] Cancel input\n";

    auto& values = m_prop.live_view_image_quality.possible;
    for (std::size_t i = 0; i < values.size(); ++i) {
        tout << '[' << i << "] " << format_live_view_image_quality(values[i]) << '\n';
    }

    tout << "[-1] Cancel input\n";
    tout << "Choose a number set a new Live View Image Quality value:\n";

    tout << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || values.size() <= selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_LiveView_Image_Quality);
    prop.SetCurrentValue(values[selected_index]);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::set_live_view_status()
{
    if (!m_prop.live_view_status.writable) {
        // Not a settable property
        tout << "Live View Status is not writable\n";
        return;
    }

    text input;
    tout << "Would you like to set a new Live View Image Quality value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << "Choose a number set a new Live View Status value:\n";
    tout << "[-1] Cancel input\n";

    tout << '[' << 1 << "] Disabled" << '\n';
    tout << '[' << 2 << "] Enabled" << '\n';

    tout << "[-1] Cancel input\n";
    tout << "Choose a number set a new Live View Image Quality value:\n";

    tout << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || 2 < selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_LiveViewStatus);
    prop.SetCurrentValue(selected_index);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt8);

    SDK::SetDeviceProperty(m_device_handle, &prop);

    get_live_view_status();
}

void CameraDevice::set_white_balance()
{
    if (!m_prop.white_balance.writable) {
        // Not a settable property
        tout << "White Balance is not writable\n";
        return;
    }

    text input;
    tout << std::endl << "Would you like to set a new White Balance value? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip setting a new value.\n";
        return;
    }

    tout << std::endl << "Choose a number set a new White Balance value:\n";
    tout << "[-1] Cancel input\n";

    auto& values = m_prop.white_balance.possible;
    for (std::size_t i = 0; i < values.size(); ++i) {
        tout << '[' << i << "] " << format_white_balance(values[i]) << '\n';
    }

    tout << "[-1] Cancel input\n";
    tout << std::endl << "Choose a number set a new White Balance value:\n";

    tout << std::endl << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0 || values.size() <= selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_WhiteBalance);
    prop.SetCurrentValue(values[selected_index]);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::execute_lock_property(CrInt16u code)
{
    load_properties();

    text input;
    tout << std::endl << "Would you like to execute Unlock or Lock? (y/n): ";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip execute a new value.\n";
        return;
    }

    tout << std::endl << "Choose a number :\n";
    tout << "[-1] Cancel input\n";

    tout << "[1] Unlock" << '\n';
    tout << "[2] Lock" << '\n';

    tout << "[-1] Cancel input\n";
    tout << "Choose a number :\n";

    tout << std::endl << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    CrInt64u ptpValue = 0;
    switch (selected_index) {
    case 1:
        ptpValue = SDK::CrLockIndicator::CrLockIndicator_Unlocked;
        break;
    case 2:
        ptpValue = SDK::CrLockIndicator::CrLockIndicator_Locked;
        break;
    default:
        selected_index = -1;
        break;
    }

    if (-1 == selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::CrDeviceProperty prop;
    prop.SetCode(code);
    prop.SetCurrentValue((CrInt64u)(ptpValue));
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

    SDK::SetDeviceProperty(m_device_handle, &prop);
}

void CameraDevice::get_af_area_position()
{
    CrInt32 num = 0;
    SDK::CrLiveViewProperty* lvProperty = NULL;
    auto err = SDK::GetLiveViewProperties(m_device_handle, &lvProperty, &num);
    if (CR_FAILED(err)) {
        tout << "GetLiveViewProperties FAILED\n";
        return;
    }
    if (lvProperty && num > 0) {
        // Got LiveViewProperty list
        for (std::int32_t i = 0; i < num; ++i) {
            auto prop = lvProperty[i];
            switch (prop.GetCode()){
                case SDK::CrLiveViewPropertyCode::CrLiveViewProperty_AF_Area_Position:
                    if (SDK::CrFrameInfoType::CrFrameInfoType_FocusFrameInfo == prop.GetFrameInfoType()) {
                        int sizVal = prop.GetValueSize();
                        int count = sizVal / sizeof(SDK::CrFocusFrameInfo);
                        SDK::CrFocusFrameInfo* pFrameInfo = (SDK::CrFocusFrameInfo*)prop.GetValue();
                        if (NULL == pFrameInfo){
                            break;
                        }
                        for (std::int32_t fram = 0; fram < count; ++fram) {
                            auto lvprop = pFrameInfo[fram];
                            char buff[512];
                            memset(buff, 0, sizeof(buff));
                            sprintf(buff, "\nFocusFrameInfo no[%d] pri[%d] w[%d] h[%d] Deno[%d-%d] Nume[%d-%d]\n",
                                fram + 1,
                                lvprop.priority,
                                lvprop.width, lvprop.height,
                                lvprop.xDenominator, lvprop.yDenominator,
                                lvprop.xNumerator, lvprop.yNumerator);
                            tout << buff;
                        }
                    }
                break;
            default:
                break;
            }
        }
    }

}

void CameraDevice::set_af_area_position()
{
    load_properties();
    // Set, FocusArea property
    tout << "Set FocusArea to Flexible_Spot_S\n";
    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FocusArea);
    prop.SetCurrentValue(SDK::CrFocusArea::CrFocusArea_Flexible_Spot_S);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);
    auto err_prop = SDK::SetDeviceProperty(m_device_handle, &prop);
    if (CR_FAILED(err_prop)) {
        tout << "FocusArea FAILED\n";
        return;
    }
    else {
        tout << "FocusArea SUCCESS\n";
    }

    std::this_thread::sleep_for(500ms);

    this->get_af_area_position();

    execute_pos_xy(SDK::CrDevicePropertyCode::CrDeviceProperty_AF_Area_Position);
}

void CameraDevice::set_select_media_format()
{
    if ((!m_prop.media_slot1_format_enable_status.writable) &&
        (!m_prop.media_slot2_format_enable_status.writable)) {
            // Not a settable property
        tout << std::endl << "Slot1 and Slot2 format enable status is not writable\n";
        return;
    }

    text input;
    tout << std::endl << "Would you like to format the media? (y/n):";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip format.\n";
        return;
    }

    tout << std::endl << "Choose a number Which media do you want to format ? \n";
    tout << "[-1] Cancel input\n";

    tout << "[1] SLOT1" << '\n';
    tout << "[2] SLOT2" << '\n';

    tout << "[-1] Cancel input\n";
    tout << "Choose a number :\n";

    tout << std::endl << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index <= 0) 
    {
        tout << "Input cancelled.\n";
        return;
    }

    CrInt64u ptpValue = 0;
    if ((1 == selected_index) && (m_prop.media_slot1_format_enable_status.writable)) {
        ptpValue = SDK::CrCommandParam::CrCommandParam_Up;
    }
    else if ((2 == selected_index) && (m_prop.media_slot2_format_enable_status.writable)) {
        ptpValue = SDK::CrCommandParam::CrCommandParam_Down;
    }
    else
    {
        tout << std::endl << "The Selected slot cannot be formatted.\n";
        return;
    }

    tout << std::endl << "All data will be deleted.Is it OK ? (y/n) \n";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip format.\n";
        return;
    }

    SDK::SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_MediaFormat, (SDK::CrCommandParam)ptpValue);

    tout << std::endl << "Formatting .....\n";

    int startflag = 0;
    while (true)
    {
        std::int32_t nprop = 0;
        SDK::CrDeviceProperty* prop_list = nullptr;
        // auto status = m_cr_lib->GetDeviceProperties(m_device_handle, &prop_list, &nprop);
        auto status = SDK::GetDeviceProperties(m_device_handle, &prop_list, &nprop);

        if (CR_FAILED(status)) {
            tout << "Failed to get device properties.\n";
            return;
        }

        
        if (prop_list && nprop > 0) {
            // Got properties list
            for (std::int32_t i = 0; i < nprop; ++i) {
                auto prop = prop_list[i];
                int nval = 0;
                

                if(SDK::CrDevicePropertyCode::CrDeviceProperty_Media_FormatProgressRate == prop.GetCode())
                {
                    if (0 < prop.GetCurrentValue() && (0 == startflag) )
                    {
                        startflag = 1;
                        break;
                    }

                    if (0 == prop.GetCurrentValue() && startflag )
                    {
                        startflag = 2;
                        tout << std::endl << "Format completed " << '\n';
                        break;
                    }
                    tout << "\r" << "FormatProgressRate:" << prop.GetCurrentValue();

                }
            }
        }
        if (startflag == 2)
        {
            break;
        }
        std::this_thread::sleep_for(500ms);
    }
}

void CameraDevice::execute_movie_rec()
{
    load_properties();

    text input;
    tout << std::endl << "Operate the movie recording button ? (y/n):";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip .\n";
        return;
    }

    tout << "Choose a number :\n";
    tout << "[-1] Cancel input\n";

    tout << "[1] Up" << '\n';
    tout << "[2] Down" << '\n';

    tout << "[-1] Cancel input\n";
    tout << "Choose a number :\n";

    tout << std::endl << "input> ";
    std::getline(tin, input);
    text_stringstream ss(input);
    int selected_index = 0;
    ss >> selected_index;

    if (selected_index < 0) {
        tout << "Input cancelled.\n";
        return;
    }

    CrInt64u ptpValue = 0;
    switch (selected_index) {
    case 1:
        ptpValue = SDK::CrCommandParam::CrCommandParam_Up;
        break;
    case 2:
        ptpValue = SDK::CrCommandParam::CrCommandParam_Down;
        break;
    default:
        selected_index = -1;
        break;
    }

    if (-1 == selected_index) {
        tout << "Input cancelled.\n";
        return;
    }

    SDK::SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_MovieRecord, (SDK::CrCommandParam)ptpValue);

}

void CameraDevice::set_custom_wb()
{
    // Set, PriorityKeySettings property
    tout << std::endl << "Set camera to PC remote";
    SDK::CrDeviceProperty priority;
    priority.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_PriorityKeySettings);
    priority.SetCurrentValue(SDK::CrPriorityKeySettings::CrPriorityKey_PCRemote);
    priority.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);
    auto err_priority = SDK::SetDeviceProperty(m_device_handle, &priority);
    if (CR_FAILED(err_priority)) {
        tout << "Priority Key setting FAILED\n";
        return;
    }
    else {
        tout << "Priority Key setting SUCCESS\n";
    }
    std::this_thread::sleep_for(500ms);
    get_position_key_setting();

    // Set, ExposureProgramMode property
    tout << std::endl << "Set the Exposure Program mode to P mode";
    SDK::CrDeviceProperty expromode;
    expromode.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_ExposureProgramMode);
    expromode.SetCurrentValue(SDK::CrExposureProgram::CrExposure_P_Auto);
    expromode.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);
    auto err_expromode = SDK::SetDeviceProperty(m_device_handle, &expromode);
    if (CR_FAILED(err_expromode)) {
        tout << "Exposure Program mode FAILED\n";
        return;
    }
    else {
        tout << "Exposure Program mode SUCCESS\n";
    }
    std::this_thread::sleep_for(500ms);
    get_exposure_program_mode();

    // Set, White Balanc property
    tout << std::endl << "Set the White Balance to Custom1\n";
    SDK::CrDeviceProperty wb;
    wb.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_WhiteBalance);
    wb.SetCurrentValue(SDK::CrWhiteBalanceSetting::CrWhiteBalance_Custom_1);
    wb.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);
    auto err_wb = SDK::SetDeviceProperty(m_device_handle, &wb);
    if (CR_FAILED(err_wb)) {
        tout << "White Balance FAILED\n";
        return;
    }
    else {
        tout << "White Balance SUCCESS\n";
    }
    std::this_thread::sleep_for(2000ms);
    get_white_balance();

    // Set, custom WB capture standby 
    tout << std::endl << "Set custom WB capture standby " << std::endl;

    bool execStat = false;
    while (false == execStat) {
        execute_downup_property(SDK::CrDevicePropertyCode::CrDeviceProperty_CustomWB_Capture_Standby);
        std::this_thread::sleep_for(2000ms);
        tout << std::endl;
        execStat = get_custom_wb();
    }

    // Set, custom WB capture 
    tout << std::endl << "Set custom WB capture ";
    execute_pos_xy(SDK::CrDevicePropertyCode::CrDeviceProperty_CustomWB_Capture);

    std::this_thread::sleep_for(5000ms);

    // Set, custom WB capture standby cancel 
    text input;
    tout << std::endl << "Set custom WB capture standby cancel. Please enter something. " << std::endl;
    std::getline(tin, input);
    if (0 == input.size() || 0 < input.size()) {
        execute_downup_property(SDK::CrDevicePropertyCode::CrDeviceProperty_CustomWB_Capture_Standby_Cancel);
        get_custom_wb();
        tout << std::endl << "Finish custom WB capture\n";
    }
    else
    {
        tout << std::endl << "Did not finish normally\n";
    }
}

void CameraDevice::set_zoom_operation()
{
    load_properties();

    text input;
    tout << std::endl << "Operate the zoom ? (y/n):";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip .\n";
        return;
    }

    while (true)
    {
        tout << std::endl << "Choose a number :\n";
        tout << "[-1] Cancel input\n";

        tout << "[1] Wide" << '\n';
        tout << "[2] Tele" << '\n';

        tout << "[-1] Cancel input\n";
        tout << "Choose a number :\n";

        tout << std::endl << "input> ";
        std::getline(tin, input);
        text_stringstream ss(input);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0) {
            tout << "Input cancelled.\n";
            return;
        }

        CrInt64u ptpValue = 0;
        switch (selected_index) {
        case 1:
            ptpValue = SDK::CrZoomOperation::CrZoomOperation_Wide;
            break;
        case 2:
            ptpValue = SDK::CrZoomOperation::CrZoomOperation_Tele;
            break;
        default:
            selected_index = -1;
            break;
        }

        if (-1 == selected_index) {
            tout << "Input cancelled.\n";
            return;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_Zoom_Operation);
        prop.SetCurrentValue(ptpValue);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);
        SDK::SetDeviceProperty(m_device_handle, &prop);
        get_zoom_operation();
        std::this_thread::sleep_for(10000ms);

        // Stop
        prop.SetCurrentValue(SDK::CrZoomOperation::CrZoomOperation_Stop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
    }
}

void CameraDevice::execute_downup_property(CrInt16u code)
{
    SDK::CrDeviceProperty prop;
    prop.SetCode(code);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

    // Down
    prop.SetCurrentValue(SDK::CrPropertyCustomWBCaptureButton::CrPropertyCustomWBCapture_Down);
    SDK::SetDeviceProperty(m_device_handle, &prop);

    std::this_thread::sleep_for(500ms);

    // Up
    prop.SetCurrentValue(SDK::CrPropertyCustomWBCaptureButton::CrPropertyCustomWBCapture_Up);
    SDK::SetDeviceProperty(m_device_handle, &prop);

    std::this_thread::sleep_for(500ms);
}

void CameraDevice::execute_pos_xy(CrInt16u code)
{
    load_properties();

    text input;
    tout << std::endl << "Change position ? (y/n):";
    std::getline(tin, input);
    if (input != TEXT("y")) {
        tout << "Skip.\n";
        return;
    }

    tout << std::endl << "Set the value of X (decimal)" << std::endl << "(The range of X is 0 to 639 (0x027F)) " << '\n';

    tout << std::endl << "input X> ";
    std::getline(tin, input);
    text_stringstream ss1(input);
    CrInt32u x = 0;
    ss1 >> x;

    if (x < 0 || x > 639) {
        tout << "Input cancelled.\n";
        return;
    }

    tout << "input X = " << x << '\n';

    std::this_thread::sleep_for(1000ms);

    tout << std::endl << "Set the value of Y (decimal)" << std::endl << "(The range of Y is 0 to 479 (0x01DF)) " << '\n';

    tout << std::endl << "input Y> ";
    std::getline(tin, input);
    text_stringstream ss2(input);
    CrInt32u y = 0;
    ss2 >> y;

    if (y < 0 || y > 479 ) {
        tout << "Input cancelled.\n";
        return;
    }

    tout << "input Y = "<< y << '\n';

    std::this_thread::sleep_for(1000ms);

    int x_y = x << 16 | y;

    tout << std::endl << "input X_Y = 0x" << std::hex << x_y << std::dec << '\n';

    SDK::CrDeviceProperty prop;
    prop.SetCode(code);
    prop.SetCurrentValue((CrInt64u)x_y);
    prop.SetValueType(SDK::CrDataType::CrDataType_UInt32);

    SDK::SetDeviceProperty(m_device_handle, &prop);
}

bool CameraDevice::connected() const
{
    return m_connected.load();
}

std::uint32_t CameraDevice::ip_address() const
{
    if (m_conn_type == ConnectionType::NETWORK)
        return m_net_info.ip_address;
    return 0;
}

text CameraDevice::mac_address() const
{
    if (m_conn_type == ConnectionType::NETWORK)
        return m_net_info.mac_address;
    return text(TEXT("N/A"));
}

std::int16_t CameraDevice::pid() const
{
    if (m_conn_type == ConnectionType::USB)
        return m_usb_info.pid;
    return 0;
}

void CameraDevice::OnConnected(SDK::DeviceConnectionVersioin version)
{
    m_connected.store(true);
    tout << "Connected to " << m_info->GetModel() << '\n';
}

void CameraDevice::OnDisconnected(CrInt32u error)
{
    m_connected.store(false);
    m_device_handle = 0;
    tout << "Disconnected from " << m_info->GetModel() << '\n';
}

void CameraDevice::OnPropertyChanged()
{
    // tout << "Property changed.\n";
}

void CameraDevice::OnLvPropertyChanged()
{
    // tout << "LvProperty changed.\n";
}

void CameraDevice::OnCompleteDownload(CrChar* filename)
{
    text file(filename);
    tout << "Complete download. File: " << file.data() << '\n';
}

void CameraDevice::OnWarning(CrInt32u warning)
{
    if (warning == SDK::CrWarning_Connect_Reconnecting) {
        tout << "Device Disconnected. Reconnecting...\n";
        return;
    }
    tout << std::endl << "Warning: 0x" << std::hex << warning << std::dec << '\n';
}

void CameraDevice::OnError(CrInt32u error)
{
    if (error == SDK::CrError_Connect_TimeOut) {
        tout << "Device Disconnect Timeout.\n Please input '0' after Connect camera\n";
        return;
    }
    tout << "Error: 0x" << std::hex << error << std::dec << '\n';
}

void CameraDevice::load_properties()
{
    std::int32_t nprop = 0;
    SDK::CrDeviceProperty* prop_list = nullptr;
    // auto status = m_cr_lib->GetDeviceProperties(m_device_handle, &prop_list, &nprop);
    auto status = SDK::GetDeviceProperties(m_device_handle, &prop_list, &nprop);

    if (CR_FAILED(status)) {
        tout << "Failed to get device properties.\n";
        return;
    }

    if (prop_list && nprop > 0) {
        // Got properties list
        for (std::int32_t i = 0; i < nprop; ++i) {
            auto prop = prop_list[i];
            int nval = 0;

            switch (prop.GetCode()) {
            case SDK::CrDevicePropertyCode::CrDeviceProperty_FNumber:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.f_number.writable = prop.IsSetEnableCurrentValue();
                m_prop.f_number.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                if (nval != m_prop.f_number.possible.size()) {
                    auto parsed_values = parse_f_number(prop.GetValues(), nval);
                    m_prop.f_number.possible.swap(parsed_values);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_IsoSensitivity:
                nval = prop.GetValueSize() / sizeof(std::uint32_t);
                m_prop.iso_sensitivity.writable = prop.IsSetEnableCurrentValue();
                m_prop.iso_sensitivity.current = static_cast<std::uint32_t>(prop.GetCurrentValue());
                if (nval != m_prop.iso_sensitivity.possible.size()) {
                    auto parsed_values = parse_iso_sensitivity(prop.GetValues(), nval);
                    m_prop.iso_sensitivity.possible.swap(parsed_values);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_ShutterSpeed:
                nval = prop.GetValueSize() / sizeof(std::uint32_t);
                m_prop.shutter_speed.writable = prop.IsSetEnableCurrentValue();
                m_prop.shutter_speed.current = static_cast<std::uint32_t>(prop.GetCurrentValue());
                if (nval != m_prop.shutter_speed.possible.size()) {
                    auto parsed_values = parse_shutter_speed(prop.GetValues(), nval);
                    m_prop.shutter_speed.possible.swap(parsed_values);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_PriorityKeySettings:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.position_key_setting.writable = prop.IsSetEnableCurrentValue();
                m_prop.position_key_setting.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                if (nval != m_prop.position_key_setting.possible.size()) {
                    auto parsed_values = parse_position_key_setting(prop.GetValues(), nval);
                    m_prop.position_key_setting.possible.swap(parsed_values);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_ExposureProgramMode:
                nval = prop.GetValueSize() / sizeof(std::uint32_t);
                m_prop.exposure_program_mode.writable = prop.IsSetEnableCurrentValue();
                m_prop.exposure_program_mode.current = static_cast<std::uint32_t>(prop.GetCurrentValue());
                if (nval != m_prop.exposure_program_mode.possible.size()) {
                    auto parsed_values = parse_exposure_program_mode(prop.GetValues(), nval);
                    m_prop.exposure_program_mode.possible.swap(parsed_values);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_DriveMode:
                nval = prop.GetValueSize() / sizeof(std::uint32_t);
                m_prop.still_capture_mode.writable = prop.IsSetEnableCurrentValue();
                m_prop.still_capture_mode.current = static_cast<std::uint32_t>(prop.GetCurrentValue());
                if (nval != m_prop.still_capture_mode.possible.size()) {
                    auto parsed_values = parse_still_capture_mode(prop.GetValues(), nval);
                    m_prop.still_capture_mode.possible.swap(parsed_values);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_FocusMode:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.focus_mode.writable = prop.IsSetEnableCurrentValue();
                m_prop.focus_mode.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                if (nval != m_prop.focus_mode.possible.size()) {
                    auto parsed_values = parse_focus_mode(prop.GetValues(), nval);
                    m_prop.focus_mode.possible.swap(parsed_values);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_LiveView_Image_Quality:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.live_view_image_quality.writable = prop.IsSetEnableCurrentValue();
                m_prop.live_view_image_quality.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                if (nval != m_prop.live_view_image_quality.possible.size()) {
                    std::vector<uint16_t> view = parse_live_view_image_quality(prop.GetValues(), nval);
                    m_prop.live_view_image_quality.possible.swap(view);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_LiveViewStatus:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.live_view_status.writable = prop.IsSetEnableCurrentValue();
                m_prop.live_view_status.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_MediaSLOT1_FormatEnableStatus:
                nval = prop.GetValueSize() / sizeof(std::uint8_t);
                m_prop.media_slot1_format_enable_status.writable = prop.IsSetEnableCurrentValue();
                m_prop.media_slot1_format_enable_status.current = static_cast<std::uint8_t>(prop.GetCurrentValue());
                if (nval != m_prop.media_slot1_format_enable_status.possible.size()) {
                    std::vector<uint8_t> mode = parse_media_slot1_format_enable_status(prop.GetValues(), nval);
                    m_prop.media_slot1_format_enable_status.possible.swap(mode);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_MediaSLOT2_FormatEnableStatus:
                nval = prop.GetValueSize() / sizeof(std::uint8_t);
                m_prop.media_slot2_format_enable_status.writable = prop.IsSetEnableCurrentValue();
                m_prop.media_slot2_format_enable_status.current = static_cast<std::uint8_t>(prop.GetCurrentValue());
                if (nval != m_prop.media_slot2_format_enable_status.possible.size()) {
                    std::vector<uint8_t> mode = parse_media_slot2_format_enable_status(prop.GetValues(), nval);
                    m_prop.media_slot2_format_enable_status.possible.swap(mode);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_WhiteBalance:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.white_balance.writable = prop.IsSetEnableCurrentValue();
                m_prop.white_balance.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                if (nval != m_prop.white_balance.possible.size()) {
                    std::vector<uint16_t> mode = parse_white_balance(prop.GetValues(), nval);
                    m_prop.white_balance.possible.swap(mode);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_CustomWB_Capture_Standby:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.customwb_capture_stanby.writable = prop.IsSetEnableCurrentValue();
                m_prop.customwb_capture_stanby.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                if (nval != m_prop.white_balance.possible.size()) {
                    std::vector<uint16_t> mode = parse_customwb_capture_stanby(prop.GetValues(), nval);
                    m_prop.customwb_capture_stanby.possible.swap(mode);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_CustomWB_Capture_Standby_Cancel:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.customwb_capture_stanby_cancel.writable = prop.IsSetEnableCurrentValue();
                m_prop.customwb_capture_stanby_cancel.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                if (nval != m_prop.customwb_capture_stanby_cancel.possible.size()) {
                    std::vector<uint16_t> mode = parse_customwb_capture_stanby_cancel(prop.GetValues(), nval);
                    m_prop.customwb_capture_stanby_cancel.possible.swap(mode);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_CustomWB_Capture_Operation:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.customwb_capture_operation.writable = prop.IsSetEnableCurrentValue();
                m_prop.customwb_capture_operation.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                if (nval != m_prop.customwb_capture_operation.possible.size()) {
                    std::vector<uint16_t> mode = parse_customwb_capture_operation(prop.GetValues(), nval);
                    m_prop.customwb_capture_operation.possible.swap(mode);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_CustomWB_Execution_State:
                nval = prop.GetValueSize() / sizeof(std::uint16_t);
                m_prop.customwb_capture_execution_state.writable = prop.IsSetEnableCurrentValue();
                m_prop.customwb_capture_execution_state.current = static_cast<std::uint16_t>(prop.GetCurrentValue());
                if (nval != m_prop.customwb_capture_execution_state.possible.size()) {
                    std::vector<uint16_t> mode = parse_customwb_capture_execution_state(prop.GetValues(), nval);
                    m_prop.customwb_capture_execution_state.possible.swap(mode);                  
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_Zoom_Operation_Status:
                nval = prop.GetValueSize() / sizeof(std::uint8_t);
                m_prop.zoom_operation_status.writable = prop.IsSetEnableCurrentValue();
                m_prop.zoom_operation_status.current = static_cast<std::uint8_t>(prop.GetCurrentValue());
                if (nval != m_prop.zoom_operation_status.possible.size()) {
                    std::vector<uint8_t> mode = parse_zoom_operation_status(prop.GetValues(), nval);
                    m_prop.zoom_operation_status.possible.swap(mode);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_Zoom_Setting:
                nval = prop.GetValueSize() / sizeof(std::uint8_t);
                m_prop.zoom_setting_type.writable = prop.IsSetEnableCurrentValue();
                m_prop.zoom_setting_type.current = static_cast<std::uint8_t>(prop.GetCurrentValue());
                if (nval != m_prop.zoom_setting_type.possible.size()) {
                    std::vector<uint8_t> mode = parse_zoom_setting_type(prop.GetValues(), nval);
                    m_prop.zoom_setting_type.possible.swap(mode);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_Zoom_Type_Status:
                nval = prop.GetValueSize() / sizeof(std::uint8_t);
                m_prop.zoom_types_status.writable = prop.IsSetEnableCurrentValue();
                m_prop.zoom_types_status.current = static_cast<std::uint8_t>(prop.GetCurrentValue());
                if (nval != m_prop.zoom_types_status.possible.size()) {
                    std::vector<uint8_t> mode = parse_zoom_types_status(prop.GetValues(), nval);
                    m_prop.zoom_types_status.possible.swap(mode);
                }
                break;
            case SDK::CrDevicePropertyCode::CrDeviceProperty_Zoom_Operation:
                nval = prop.GetValueSize() / sizeof(std::uint8_t);
                m_prop.zoom_operation.writable = prop.IsSetEnableCurrentValue();
                m_prop.zoom_operation.current = static_cast<std::uint8_t>(prop.GetCurrentValue());
                if (nval != m_prop.zoom_operation.possible.size()) {
                    std::vector<uint8_t> mode = parse_zoom_operation(prop.GetValues(), nval);
                    m_prop.zoom_operation.possible.swap(mode);
                }
                break;

            default:
                break;
            }
        }
    }
}

void CameraDevice::get_property(SDK::CrDeviceProperty& prop) const
{
    SDK::CrDeviceProperty* properties = nullptr;
    int nprops = 0;
    // m_cr_lib->GetDeviceProperties(m_device_handle, &properties, &nprops);
    SDK::GetDeviceProperties(m_device_handle, &properties, &nprops);
}

bool CameraDevice::set_property(SDK::CrDeviceProperty& prop) const
{
    // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
    SDK::SetDeviceProperty(m_device_handle, &prop);
    return false;
}

} // namespace cli
