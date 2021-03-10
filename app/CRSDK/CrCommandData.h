#ifndef CRCOMMANDDATA_H
#define CRCOMMANDDATA_H

#include "CrTypes.h"

namespace SCRSDK
{
enum CrCommandId
{
	CrCommandId_Release = 0,
	CrCommandId_MovieRecord,
	CrCommandId_CancelShooting,
	CrCommandId_MediaFormat = 4,
};

enum CrCommandParam : CrInt16u
{
	CrCommandParam_Up = 0x0000,
	CrCommandParam_Down = 0x0001,
};
}

#endif // CRCOMMANDDATA_H
