OPENCV_VERSION=4.1.0

apt-get update 
apt-get install -y build-essential 
apt-get install -y cmake 
apt-get install -y wget 
apt-get install -y git 
apt-get install -y unzip 
apt-get install -y yasm 
apt-get install -y pkg-config 
apt-get install -y libjpeg-dev 
apt-get install -y libtiff-dev 
apt-get install -y libpng-dev 
apt-get install -y libavcodec-dev 
apt-get install -y libavformat-dev 
apt-get install -y libswscale-dev 
apt-get install -y libv4l-dev 
apt-get install -y libatlas-base-dev 
apt-get install -y gfortran 
apt-get install -y libtbb2 
apt-get install -y libtbb-dev 
apt-get install -y libpq-dev 
apt-get install -y libgtk2.0-dev 

cd /
wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip
unzip ${OPENCV_VERSION}.zip
wget https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip -O ${OPENCV_VERSION}-contrib.zip 
unzip ${OPENCV_VERSION}-contrib.zip
mkdir /opencv-${OPENCV_VERSION}/cmake_binary
cd /opencv-${OPENCV_VERSION}/cmake_binary \
&& cmake -DBUILD_TIFF=ON \
  -DBUILD_opencv_java=OFF \
  -DWITH_CUDA=OFF \
  -DENABLE_AVX=ON \
  -DWITH_OPENGL=ON \
  -DWITH_OPENCL=ON \
  -DWITH_IPP=ON \
  -DWITH_TBB=ON \
  -DWITH_EIGEN=ON \
  -DWITH_V4L=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=RELEASE \
  -DCMAKE_INSTALL_PREFIX=$(python3.5 -c "import sys; print(sys.prefix)") \
  -DOPENCV_EXTRA_MODULES_PATH=/opencv_contrib-${OPENCV_VERSION}/modules \
  -DOPENCV_GENERATE_PKGCONFIG=YES \
  .. 
make -j$(nproc) install
cp /opencv-4.1.0/cmake_binary/unix-install/opencv4.pc /usr/lib/pkgconfig/
rm /${OPENCV_VERSION}.zip
rm /${OPENCV_VERSION}-contrib.zip
