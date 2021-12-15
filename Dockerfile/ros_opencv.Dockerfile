# Compile opencv against ROS
FROM ros:galactic

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  wget \
  && rm -rf /var/lib/apt/lists/*

# Build opencv
RUN wget -O opencv.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.5.2.tar.gz \
  && tar -xzf opencv.tar.gz \
  && rm opencv.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE:STRING=Release \
    -D CMAKE_INSTALL_PREFIX:STRING=/opt/opencv \
    -D BUILD_LIST:STRING=core,imgproc,calib3d,,imgcodecs \
    -D BUILD_TESTS:BOOL=OFF \
    -D BUILD_PERF_TESTS:BOOL=OFF \
    -D BUILD_EXAMPLES:BOOL=OFF \
    -D BUILD_opencv_apps=OFF \
    -D WITH_1394:BOOL=OFF \
    -D WITH_ADE:BOOL=OFF \
    -D WITH_ARAVIS:BOOL=OFF \
    -D WITH_CLP:BOOL=OFF \
    -D WITH_CUDA:BOOL=OFF \
    -D WITH_EIGEN:BOOL=OFF \
    -D WITH_FFMPEG:BOOL=OFF \
    -D WITH_FREETYPE:BOOL=OFF \
    -D WITH_GDAL:BOOL=OFF \
    -D WITH_GDCM:BOOL=OFF \
    -D WITH_GPHOTO2:BOOL=OFF \
    -D WITH_GSTREAMER:BOOL=OFF \
    -D WITH_GTK:BOOL=OFF \
    -D WITH_GTK_2_X:BOOL=OFF \
    -D WITH_HALIDE:BOOL=OFF \
    -D WITH_HPX:BOOL=OFF \
    -D WITH_IMGCODEC_HDR:BOOL=OFF \
    -D WITH_IMGCODEC_PFM:BOOL=OFF \
    -D WITH_IMGCODEC_PXM:BOOL=OFF \
    -D WITH_IMGCODEC_SUNRASTER:BOOL=OFF \
    -D WITH_INF_ENGINE:BOOL=OFF \
    -D WITH_IPP:BOOL=OFF \
    -D WITH_ITT:BOOL=OFF \
    -D WITH_JASPER:BOOL=OFF \
    -D WITH_JPEG:BOOL=OFF \
    -D WITH_LAPACK:BOOL=OFF \
    -D WITH_LIBREALSENSE:BOOL=OFF \
    -D WITH_MFX:BOOL=OFF \
    -D WITH_NGRAPH:BOOL=OFF \
    -D WITH_ONNX:BOOL=OFF \
    -D WITH_OPENCL:BOOL=OFF \
    -D WITH_OPENCLAMDBLAS:BOOL=OFF \
    -D WITH_OPENCLAMDFFT:BOOL=OFF \
    -D WITH_OPENCL_SVM:BOOL=OFF \
    -D WITH_OPENEXR:BOOL=OFF \
    -D WITH_OPENGL:BOOL=OFF \
    -D WITH_OPENJPEG:BOOL=OFF \
    -D WITH_OPENMP:BOOL=OFF \
    -D WITH_OPENNI:BOOL=OFF \
    -D WITH_OPENNI2:BOOL=OFF \
    -D WITH_OPENVX:BOOL=OFF \
    -D WITH_PLAIDML:BOOL=OFF \
    -D WITH_PNG:BOOL=OFF \
    -D WITH_PROTOBUF:BOOL=OFF \
    -D WITH_PTHREADS_PF:BOOL=OFF \
    -D WITH_PVAPI:BOOL=OFF \
    -D WITH_QT:BOOL=OFF \
    -D WITH_QUIRC:BOOL=OFF \
    -D WITH_TBB:BOOL=OFF \
    -D WITH_TIFF:BOOL=OFF \
    -D WITH_UEYE:BOOL=OFF \
    -D WITH_V4L:BOOL=OFF \
    -D WITH_VA:BOOL=OFF \
    -D WITH_VA_INTEL:BOOL=OFF \
    -D WITH_VTK:BOOL=OFF \
    -D WITH_VULKAN:BOOL=OFF \
    -D WITH_WEBP:BOOL=OFF \
    -D WITH_XIMEA:BOOL=OFF \
    -D WITH_XINE:BOOL=OFF \
    -S opencv-4.5.2/ \
    -B build/ \
  && cmake --build build/ --target install \
  && rm -r opencv-4.5.2 build

## basler
# linux/amd64 or linux/arm64
ARG TARGETPLATFORM
#COPY ./pylon_6.2.0.21487_x86_64_setup.tar.gz ./pylon.tar.gz

# Install basler pylon6.2
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
    wget -O pylon.tar.gz https://www.baslerweb.com/fp-1615275588/media/a_b_testing_1/pylon_6.2.0.21487_x86_64_setup.tar.gz \
    && tar -xzvf pylon*.tar.gz \
    tar -xzvf pylon*.tar.gz \
    && mkdir /opt/pylon \
    && tar -C /opt/pylon -xzf ./pylon_*_x86_64.tar.gz \
    && chmod 755 /opt/pylon \
    && rm -r pylon* ; fi

RUN if [ "$(uname -p)" = "aarch64" ]; then \
    wget -O pylon.tar.gz https://www.baslerweb.com/fp-1615276046/media/a_b_testing_1/pylon_6.2.0.21487_aarch64_setup.tar.gz \
    && tar -xzvf pylon*.tar.gz \
    && mkdir /opt/pylon \
    && tar -C /opt/pylon -xzf ./pylon_*_aarch64.tar.gz \
    && chmod 755 /opt/pylon \
    #&& ./opt/pylon/share/pylon/setup-usb.sh \
    && rm -r pylon* ; fi

# CMD ["sh", "-c", "echo \"/opt/pylon/bin/pylon-setup-env.sh\" >> /root/.bashrc && ldconfig"]