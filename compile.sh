export PERFORMANCE_TEST_FIXTURE_ENABLE_TRACE=0

colcon build \
   --packages-ignore cyclonedds rcl_logging_log4cxx rosidl_generator_py performance_test_fixture  \
   --packages-above rcl \
   --packages-up-to rcljava \
   --cmake-args \
   -DBUILD_TESTING=0 \
   -DPYTHON_EXECUTABLE=${PYTHON3_EXEC} \
   -DPYTHON_LIBRARY=${PYTHON3_LIBRARY} \
   -DPYTHON_INCLUDE_DIR=${PYTHON3_INCLUDE_DIR} \
   -DCMAKE_TOOLCHAIN_FILE=${ANDROID_NDK}/build/cmake/android.toolchain.cmake \
   -DANDROID_FUNCTION_LEVEL_LINKING=OFF \
   -DANDROID_NATIVE_API_LEVEL=${ANDROID_NATIVE_API_LEVEL} \
   -DANDROID_TOOLCHAIN_NAME=${ANDROID_TOOLCHAIN_NAME} \
   -DANDROID_STL=c++_shared \
   -DANDROID_ABI=${ANDROID_ABI} \
   -DANDROID_NDK=${ANDROID_NDK} \
   -DTHIRDPARTY=ON \
   -DCOMPILE_EXAMPLES=OFF \
   -DFORCE_BUILD_VENDOR_PKG=ON \
   -DCMAKE_FIND_ROOT_PATH="${PWD}/install" \
   -DANDROID=ON

   #\
   #-DBUILD_TESTING=OFF \
   #-DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop \
   #-DCMAKE_FIND_ROOT_PATH="${PWD}/install"

   # -DTHIRDPARTY_android-ifaddrs=FORCE \
   # rmw_fastrtps_shared_cpp rmw_fastrtps_shared_c rmw_fastrtps_dynamic_cpp rmw_fastrtps_dynamic_c rmw_fastrtps_cpp rmw_fastrtps_c

    #  --packages-up-to rcljava \
#ament_cmake_libraries

# libyaml_vendor rcl_yaml_param_parser launch_yaml rcl_logging_spdlog
#
