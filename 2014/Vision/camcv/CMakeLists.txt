cmake_minimum_required(VERSION 2.8)
project( camcv )
SET(COMPILE_DEFINITIONS -Werror)

find_package( OpenCV REQUIRED )

include_directories(/opt/vc/userland/host_applications/linux/libs/bcm_host/include)
include_directories(/opt/vc/userland/interface/vcos)
include_directories(/opt/vc/userland)
include_directories(/opt/vc/userland/interface/vcos/pthreads)
include_directories(/opt/vc/userland/interface/vmcs_host/linux)

include_directories(/opt/vc/userland/interface/khronos/include)
include_directories(/opt/vc/userland/interface/khronos/common)

include_directories(/usr/include/opencv2/imgproc)
include_directories(/usr/include/opencv2/video)
include_directories(/usr/include/opencv2/core)

include_directories(/usr/include)

add_executable(camcv RaspiCamControl.c RaspiCLI.c RaspiPreview.c camcv.c 
RaspiTex.c RaspiTexUtil.c teapot.c models.c square.c mirror.c yuv.c sobel.c tga.c
rs232.c)

target_link_libraries(camcv /opt/vc/lib/libmmal_core.so
/opt/vc/lib/libmmal_util.so /opt/vc/lib/libmmal_vc_client.so /opt/vc/lib/libvcos.so 
/opt/vc/lib/libbcm_host.so /opt/vc/lib/libGLESv2.so /opt/vc/lib/libEGL.so
${OpenCV_LIBS})
