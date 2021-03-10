## Script for enumerating CameraRemote SDK public header files
## referenced by RemoteCli
set(__crsdk_hdr_dir ${CMAKE_CURRENT_SOURCE_DIR}/app/CRSDK)

### Enumerate CameraRemote SDK public header files ###
message("[${PROJECT_NAME}] Indexing CameraRemote SDK public header files..")
set(__crsdk_hdrs
    ${__crsdk_hdr_dir}/CameraRemote_SDK.h
    ${__crsdk_hdr_dir}/CrCommandData.h
    ${__crsdk_hdr_dir}/CrDefines.h
    ${__crsdk_hdr_dir}/CrDeviceProperty.h
    ${__crsdk_hdr_dir}/CrError.h
    ${__crsdk_hdr_dir}/CrImageDataBlock.h
    ${__crsdk_hdr_dir}/CrTypes.h
    ${__crsdk_hdr_dir}/ICrCameraObjectInfo.h
    ${__crsdk_hdr_dir}/IDeviceCallback.h
)

set(crsdk_hdrs ${__crsdk_hdrs})
source_group("CRSDK" FILES ${crsdk_hdrs})

set(crsdk_hdr_dir ${__crsdk_hdr_dir})
