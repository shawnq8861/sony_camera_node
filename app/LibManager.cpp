#include "LibManager.h"
#include <cstdlib>
#if defined(_MSC_VER)
#include <Windows.h>
#endif
#include "Text.h"

namespace impl
{
struct LibraryHandle
{

};
} // namespace impl

using LibraryHandle = impl::LibraryHandle;

namespace cli
{
CRLibInterface* load_cr_lib()
{
    CRLibInterface* cr_lib = new CRLibInterface;

#if defined(_MSC_VER)
    cr_lib->m_handle = LoadLibraryEx(_T("LjCore.dll"), NULL, LOAD_WITH_ALTERED_SEARCH_PATH);
    if (!cr_lib->m_handle) {
        tout << "Failed to load CrCore.dll. Abort.\n";
        std::exit(EXIT_FAILURE);
    }

    cr_lib->Init = CrInit(GetProcAddress(cr_lib->m_handle, "Init"));
    cr_lib->Release = CrRelease(GetProcAddress(cr_lib->m_handle, "Release"));
    cr_lib->EnumCameraObjects = CrEnumCameraObjects(GetProcAddress(cr_lib->m_handle, "EnumCameraObjects"));
    cr_lib->CreateCameraObjectInfo = CrCreateCameraObjectInfo(GetProcAddress(cr_lib->m_handle, "CreateCameraObjectInfo"));
    cr_lib->Connect = CrConnect(GetProcAddress(cr_lib->m_handle, "Connect"));
    cr_lib->Disconnect = CrDisconnect(GetProcAddress(cr_lib->m_handle, "Disconnect"));
    cr_lib->FinalizeDevice = CrFinalizeDevice(GetProcAddress(cr_lib->m_handle, "FinalizeDevice"));
    cr_lib->GetDeviceProperties = CrGetDeviceProperties(GetProcAddress(cr_lib->m_handle, "GetDeviceProperties"));
    cr_lib->ReleaseDeviceProperties = CrReleaseDeviceProperties(GetProcAddress(cr_lib->m_handle, "ReleaseDeviceProperties"));
    cr_lib->SetDeviceProperty = CrSetDeviceProperty(GetProcAddress(cr_lib->m_handle, "SetDeviceProperty"));
    cr_lib->SendCommand = CrSendCommand(GetProcAddress(cr_lib->m_handle, "SendCommond"));
    cr_lib->SetSaveInfo = CrSetSaveInfo(GetProcAddress(cr_lib->m_handle, "SetSaveInfo"));
#endif // defined(_MSC_VER)

    if (!cr_lib->Init
        || !cr_lib->Release
        || !cr_lib->EnumCameraObjects
        || !cr_lib->CreateCameraObjectInfo
        || !cr_lib->Connect
        || !cr_lib->Disconnect
        || !cr_lib->FinalizeDevice
        || !cr_lib->GetDeviceProperties
        || !cr_lib->ReleaseDeviceProperties
        || !cr_lib->SetDeviceProperty
        || !cr_lib->SendCommand
        || !cr_lib->SetSaveInfo)
    {
        tout << "CrCore.dll functions failed to load properly. Aborting.\n";
        std::exit(EXIT_FAILURE);
    }

    tout << "Successfully loaded CrCore.dll\n\n";

    return cr_lib;
}

void free_cr_lib(CRLibInterface** cr_lib)
{
#if defined(_MSC_VER)
    FreeLibrary((*cr_lib)->m_handle);
#endif // defined(_MSC_VER)
    delete *cr_lib;
    *cr_lib = nullptr;
    tout << "Unloaded CrCore library\n";
}
} // namespace cli
