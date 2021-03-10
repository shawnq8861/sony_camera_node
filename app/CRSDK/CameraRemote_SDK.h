#ifndef CAMERAREMOTE_SDK_H
#define CAMERAREMOTE_SDK_H

#if defined(WIN32) || defined(_WIN64)

	#ifdef CR_SDK_EXPORTS
		#define SCRSDK_API __declspec(dllexport)
	#else
		#define SCRSDK_API __declspec(dllimport)
	#endif

#else

	#if defined(__GNUC__)
		#ifdef CR_SDK_EXPORTS
			#define SCRSDK_API __attribute__ ((visibility ("default")))
		#else
			#define SCRSDK_API
		#endif
	#endif

#define __T(x) x

#endif

#include "CrCommandData.h"
#include "CrDefines.h"
#include "CrDeviceProperty.h"
#include "CrError.h"
#include "CrImageDataBlock.h"
#include "CrTypes.h"
#include "ICrCameraObjectInfo.h"

namespace SCRSDK
{

class IDeviceCallback;

typedef void* objeventcallback;
typedef void(*fneventcallback)(objeventcallback obj, CrInt16u eventCode, CrInt32u param1, CrInt32u /*param2*/, CrInt32u /*param3*/);

/*SDK API*/
extern "C"
SCRSDK_API
bool Init(CrInt32u logtype = 0);

extern "C"
SCRSDK_API
bool Release();

extern "C"
SCRSDK_API
// This function enumerates the cameras that are connected to the pc via the protocol and the physical connection that the library supports.
CrError EnumCameraObjects(ICrEnumCameraObjectInfo** ppEnumCameraObjectInfo, CrInt8u timeInSec = 3);

extern "C"
SCRSDK_API
ICrCameraObjectInfo* CreateCameraObjectInfo(CrChar* name, CrChar *model, CrInt16 usbPid, CrInt32u idType, CrInt32u idSize, CrInt8u* id, CrChar *connecttypename, CrChar *adaptorname, CrChar *pairingnecessity);

extern "C"
SCRSDK_API
CrError EditSDKInfo(CrInt16u infotype);

extern "C"
SCRSDK_API
// This function connects the specified camera as Remote Connect Device.
CrError Connect(/*in*/ ICrCameraObjectInfo* pCameraObjectInfo, /*in*/  IDeviceCallback* callback, /*out*/ CrDeviceHandle* deviceHandle);

extern "C"
SCRSDK_API
// This function disconnects the connection device.
CrError Disconnect(/*in*/ CrDeviceHandle deviceHandle);

extern "C"
SCRSDK_API
// This function release and finalize the device.
CrError ReleaseDevice(/*in*/ CrDeviceHandle deviceHandle);

extern "C"
SCRSDK_API
CrError GetDeviceProperties(/*in*/ CrDeviceHandle deviceHandle, /*out*/CrDeviceProperty** properties, /*out*/ CrInt32* numOfPropoties);

extern "C"
SCRSDK_API
CrError ReleaseDeviceProperties(/*in*/ CrDeviceHandle deviceHandle, /*in*/CrDeviceProperty* properties);

extern "C"
SCRSDK_API
CrError SetDeviceProperty(/*in*/ CrDeviceHandle deviceHandle, /*in*/CrDeviceProperty* pProperty);

extern "C"
SCRSDK_API
CrError SendCommand(/*in*/ CrDeviceHandle deviceHandle, /*in*/ CrInt32u commandId, /*in*/CrCommandParam commandParam);

extern "C"
SCRSDK_API
CrError GetLiveViewImage(/*in*/ CrDeviceHandle deviceHandle, /*in*/CrImageDataBlock*imageData);

extern "C"
SCRSDK_API
CrError GetLiveViewImageInfo(/*in*/ CrDeviceHandle deviceHandle, /*out*/ CrImageInfo* info);

extern "C"
SCRSDK_API
CrError GetLiveViewProperties(/*in*/ CrDeviceHandle deviceHandle, /*out*/CrLiveViewProperty** properties, /*out*/ CrInt32* numOfProperties);

extern "C"
SCRSDK_API
CrError ReleaseLiveViewProperties(/*in*/ CrDeviceHandle deviceHandle, /*in*/CrLiveViewProperty* properties);

extern "C"
SCRSDK_API
CrError GetDeviceSetting(/*in*/ CrDeviceHandle deviceHandle, /*in*/ CrInt32u key, /*out*/ CrInt32u* value);

extern "C"
SCRSDK_API
CrError SetDeviceSetting(/*in*/ CrDeviceHandle deviceHandle, /*in*/ CrInt32u key, /*out*/ CrInt32u value);

extern "C"
SCRSDK_API
CrError SetSaveInfo(/*in*/ CrDeviceHandle deviceHandle, /*in*/ CrChar *path, CrChar* prefix, CrInt32 no);

// Get SDK version constant - Determined at build time
extern "C"
SCRSDK_API
CrInt32u GetSDKVersion();

// Get SDK serial number constant
extern "C"
SCRSDK_API
CrInt32u GetSDKSerial();
}
#endif //CAMERAREMOTE_SDK_H
