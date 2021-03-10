#ifndef LIBMANAGER_H
#define LIBMANAGER_H

#include <functional>
#include "CRSDK/CameraRemote_SDK.h"

using namespace SCRSDK;

namespace cli
{
using CrInit                    = bool (*)(CrInt32u);
using CrRelease                 = bool (*)();
using CrEnumCameraObjects       = CrError(*)(ICrEnumCameraObjectInfo**, CrInt8u);
using CrCreateCameraObjectInfo  = ICrCameraObjectInfo * (*)(CrChar*, CrChar*, CrInt16, CrInt32u, CrInt32u, CrInt8u*, CrChar*, CrChar*, CrChar*);
using CrConnect                 = CrError(*)(const ICrCameraObjectInfo*, IDeviceCallback*, CrDeviceHandle*);
using CrDisconnect              = CrError(*)(CrDeviceHandle);
using CrFinalizeDevice          = CrError(*)(CrDeviceHandle);
using CrGetDeviceProperties     = CrError(*)(CrDeviceHandle, CrDeviceProperty**, int*);
using CrReleaseDeviceProperties = CrError(*)(CrDeviceHandle, CrDeviceProperty*);
using CrSetDeviceProperty       = CrError(*)(CrDeviceHandle, CrDeviceProperty*);
using CrSendCommand             = CrError(*)(CrDeviceHandle, CrInt32u, CrCommandParam);
using CrSetSaveInfo             = CrError(*)(CrDeviceHandle, CrChar*, CrChar*, CrInt32);

// Forward declare
struct LibraryHandle;

class CRLibInterface
{
public:
    CrInit Init;
    CrRelease Release;
    CrEnumCameraObjects EnumCameraObjects;
    CrCreateCameraObjectInfo CreateCameraObjectInfo;
    CrConnect Connect;
    CrDisconnect Disconnect;
    CrFinalizeDevice FinalizeDevice;
    CrGetDeviceProperties GetDeviceProperties;
    CrReleaseDeviceProperties ReleaseDeviceProperties;
    CrSetDeviceProperty SetDeviceProperty;
    CrSendCommand SendCommand;
    CrSetSaveInfo SetSaveInfo;

private:
    LibraryHandle* m_handle;
};

CRLibInterface* load_cr_lib();
void free_cr_lib(CRLibInterface** cr_lib);
} // namespace cli

#endif // !LIBMANAGER_H
