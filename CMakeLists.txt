cmake_minimum_required(VERSION 3.5)
project (rtdp)

set(SOURCES src/Hello.cpp src/main.cpp src/rtdp.cpp src/ssp.cpp)
set(LIBS_ARMA		"armadillo")  #lapack blas
set(LIBS_NETCDF_C++	"netcdf_c++4")
set(LIBS_YAML_CPP	"yaml-cpp")
set(LIBS_SOGP		"sogp")
set(LIBS_OPENGL		"GLU glut GL Xext pthread")
set(LIBS_OPENCV		"opencv_core opencv_highgui opencv_imgproc opencv_video") #-lopencv_imgcodecs
set(LIBS_BOOST_FILESYSTEM  "boost_filesystem")
set(LIBS_BOOST_SYSTEM      "boost_system")
set(LIBS_ALL
  #${LIBS_ARMA}
  #${LIBS_NETCDF_C++} ${LIBS_YAML_CPP} ${LIBS_GP}
  #${LIBS_OPENGL} ${LIBS_OPENCV}
)

set(EXTERNAL_LIBS_PATH
  "/usr/local/lib"
  "/usr/lib"
)

set(SOGP_INCL_PATH
  "/usr/local/include/sogp"
)

set(EIGEN_INCL_PATH
  "/usr/local/include/eigen3"
  "/usr/include/eigen3"
)

add_executable(rtdp ${SOURCES})

target_include_directories(rtdp PRIVATE ${PROJECT_SOURCE_DIR}/include
${EXTERNAL_LIBS_PATH} ${EIGEN_INCL_PATH} ${SOGP_INCL_PATH} ${LIBS_BOOST_SYSTEM} ${LIBS_BOOST_FILESYSTEM})
