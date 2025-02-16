 export PERFORMANCE_TEST_FIXTURE_ENABLE_TRACE=0
 export ANDROID_NDK="$HOME/Android/Sdk/ndk/27.0.12077973"
 export ANDROID_HOME="$HOME/Android/Sdk"
 export PYTHON3_EXEC="$( which python3 )"
 export PYTHON3_LIBRARY="$( ${PYTHON3_EXEC} -c 'import os.path; from distutils import sysconfig; print(os.path.realpath(os.path.join(sysconfig.get_config_var("LIBPL"), sysconfig.get_config_var("LDLIBRARY"))))' )"
 export PYTHON3_INCLUDE_DIR="$( ${PYTHON3_EXEC} -c 'from distutils import sysconfig; print(sysconfig.get_config_var("INCLUDEPY"))' )"
 export ANDROID_ABI=arm64-v8a
 export ANDROID_NATIVE_API_LEVEL=android-28
 export ANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang

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
