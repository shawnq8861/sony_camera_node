#ifndef CRDEFINES_H
#define CRDEFINES_H

#include "CrTypes.h"
#include "CrError.h"

namespace SCRSDK
{
	typedef CrInt64 CrDeviceHandle;

	typedef CrInt64 CrDeviceFeature;

	typedef CrInt32u CrImageID;

	typedef CrInt32u CrImageType;

	enum CrDataType : CrInt32u
	{
		CrDataType_Undefined	= 0x0000,
		CrDataType_UInt8		= 0x0001,
		CrDataType_UInt16		= 0x0002,
		CrDataType_UInt32		= 0x0003,
		CrDataType_UInt64		= 0x0004,
		CrDataType_UInt128		= 0x0005,
		CrDataType_SignBit		= 0x1000,
		CrDataType_Int8			= CrDataType_SignBit | CrDataType_UInt8,
		CrDataType_Int16		= CrDataType_SignBit | CrDataType_UInt16,
		CrDataType_Int32		= CrDataType_SignBit | CrDataType_UInt32,
		CrDataType_Int64		= CrDataType_SignBit | CrDataType_UInt64,
		CrDataType_Int128		= CrDataType_SignBit | CrDataType_UInt128,
		CrDataType_ArrayBit		= 0x2000,
		CrDataType_UInt8Array	= CrDataType_ArrayBit | CrDataType_UInt8,
		CrDataType_UInt16Array	= CrDataType_ArrayBit | CrDataType_UInt16,
		CrDataType_UInt32Array	= CrDataType_ArrayBit | CrDataType_UInt32,
		CrDataType_UInt64Array	= CrDataType_ArrayBit | CrDataType_UInt64,
		CrDataType_UInt128Array	= CrDataType_ArrayBit | CrDataType_UInt128,
		CrDataType_Int8Array	= CrDataType_ArrayBit | CrDataType_Int8,
		CrDataType_Int16Array	= CrDataType_ArrayBit | CrDataType_Int16,
		CrDataType_Int32Array	= CrDataType_ArrayBit | CrDataType_Int32,
		CrDataType_Int64Array	= CrDataType_ArrayBit | CrDataType_Int64,
		CrDataType_Int128Array	= CrDataType_ArrayBit | CrDataType_Int128,
		CrDataType_RangeBit		= 0x4000,
		CrDataType_UInt8Range	= CrDataType_RangeBit | CrDataType_UInt8,
		CrDataType_UInt16Range	= CrDataType_RangeBit | CrDataType_UInt16,
		CrDataType_UInt32Range	= CrDataType_RangeBit | CrDataType_UInt32,
		CrDataType_UInt64Range	= CrDataType_RangeBit | CrDataType_UInt64,
		CrDataType_UInt128Range	= CrDataType_RangeBit | CrDataType_UInt128,
		CrDataType_Int8Range	= CrDataType_RangeBit | CrDataType_Int8,
		CrDataType_Int16Range	= CrDataType_RangeBit | CrDataType_Int16,
		CrDataType_Int32Range	= CrDataType_RangeBit | CrDataType_Int32,
		CrDataType_Int64Range	= CrDataType_RangeBit | CrDataType_Int64,
		CrDataType_Int128Range	= CrDataType_RangeBit | CrDataType_Int128,
        CrDataType_STR          = 0xFFFF,

	};
	
	#define CrChanged 0x0001

	typedef enum
	{
		DEVICE_CONNECTION_VERSION_UNKNOWN = 0,
		DEVICE_CONNECTION_VERSION_RCP3 = 300,
	}DeviceConnectionVersioin;

	typedef enum
	{
		Setting_Key_EnableLiveView = 0,
	}SettingKey;
}

#endif //CRDEFINES_H
