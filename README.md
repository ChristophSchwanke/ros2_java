# ROS 2 Java client library

### Build status

| Target                                    | Status        |
|-------------------------------------------|---------------|
| **ROS Galactic - Ubuntu Focal (OpenJDK)** | ![Build Status](https://github.com/ros2-java/ros2_java/workflows/CI/badge.svg?branch=main) |

## Introduction

This is a set of projects (bindings, code generator, examples and more) that enables developers to write ROS 2
applications for the JVM and Android.

Besides this repository itself, there's also:
- https://github.com/ros2-java/ament_java, which adds support for Gradle to Ament
- https://github.com/ros2-java/ament_gradle_plugin, a Gradle plugin that makes it easier to use ROS 2 in Java and Android project. The Gradle plugin can be installed from Gradle Central https://plugins.gradle.org/plugin/org.ros2.tools.gradle
- https://github.com/ros2-java/ros2_java_examples, examples for the Java Runtime Environment
- https://github.com/ros2-java/ros2_android_examples, examples for Android

### Is this Java only?

No, any language that targets the JVM can be used to write ROS 2 applications.

### Including Android?

Yep! Make sure to use Fast-RTPS as your DDS vendor and at least [this revision](https://github.com/eProsima/Fast-RTPS/commit/5301ef203d45528a083821c3ba582164d782360b).

### Features

The current set of features include:
- Generation of all builtin and complex ROS types, including arrays, strings, nested types, constants, etc.
- Support for publishers and subscriptions
- Tunable Quality of Service (e.g. lossy networks, reliable delivery, etc.)
- Clients and services
- Timers
- Composition (i.e. more than one node per process)
- Time handling (system and steady, ROS time not yet supported https://github.com/ros2-java/ros2_java/issues/122)
- Support for Android
- Parameters services and clients (both asynchronous and synchronous)

## Sounds great, how can I try this out?

Note that "Install dependencies" applies for Java 2 on Desktop (x86) and Android (arm-ABI), whereas the following sections are exclusive for their respective architecture.

### Install dependencies

> Note: While the following instructions use a Linux shell the same can be done on other platforms like Windows with slightly adjusted commands. Java, the JDK and Gradle can also be done via Android Studio.

1. Only Java Desktop (not Android): [Install ROS 2](https://index.ros.org/doc/ros2/Installation).

1. Install Java and a JDK.

    On Ubuntu, you can install OpenJDK with:

        sudo apt install default-jdk

1. Install Gradle.
Make sure you have Gradle 3.2 (or later) installed.

    *Ubuntu Bionic or later*

        sudo apt install gradle

    *macOS*

        brew install gradle

    Note: if run into compatibily issues between gradle 3.x and Java 9, try using Java 8,

        brew tap caskroom/versions
        brew cask install java8
        export JAVA_HOME=/Library/Java/JavaVirtualMachines/1.8.0.jdk/Contents/Home

    *Windows*

        choco install gradle

1. Install build tools:

        sudo apt install curl python3-colcon-common-extensions python3-pip python3-vcstool

1. Install Gradle extensions for colcon:

        python3 -m pip install -U git+https://github.com/colcon/colcon-gradle
        python3 -m pip install --no-deps -U git+https://github.com/colcon/colcon-ros-gradle

### Download and Build ROS 2 Java for Desktop (see Android below)

Do not execute these steps, if you want to compile for Android / arm-ABI - see below. If you wanted to compile for Android, but already did some steps for Desktop Java (x86), make sure to delete all folders for a fresh start.

1. Source your ROS 2 installation, for example:

        source /opt/ros/galactic/setup.bash

1. Download the ROS 2 Java repositories into a workspace:

        mkdir -p ros2_java_ws/src
        cd ros2_java_ws
        curl -skL https://raw.githubusercontent.com/ros2-java/ros2_java/main/ros2_java_desktop.repos | vcs import src

1. **Linux only** Install ROS dependencies:

        rosdep install --from-paths src -y -i --skip-keys "ament_tools"

1. Build desktop packages:

        colcon build --symlink-install

    *Note, on Windows we have to use `--merge-install`*

        colcon build --merge-install


### Download and Build ROS 2 Java for Android

To compile for Android perform the steps in "Install dependencies". The Android setup is slightly more complex, you'll need the SDK and NDK installed, and an Android device where you can run the examples.

Although the `ros2_java_android.repos` file contains all the repositories for the Android bindings to compile, we'll have to disable certain packages (`python_cmake_module`, `rosidl_generator_py`, `test_msgs`) that are included the repositories and that we either don't need or can't cross-compile properly (e.g. the Python generator)

1. Clone ROS 2 and ROS 2 Java source code:

        mkdir -p $HOME/ros2_android_ws/src
        cd $HOME/ros2_android_ws
        curl https://raw.githubusercontent.com/ChristophSchwanke/ros2_java/main/ros2_java_android.repos > repo.repos
        curl https://raw.githubusercontent.com/ChristophSchwanke/ros2_java/main/compile.sh > compile.sh
        curl https://raw.githubusercontent.com/ChristophSchwanke/ros2_java/main/updateGit.sh > updateGit.sh
        ./updateGit.sh

1. Possibly edit Android build configuration in compile.sh:

        export PYTHON3_EXEC="$( which python3 )"
        export PYTHON3_LIBRARY="$( ${PYTHON3_EXEC} -c 'import os.path; from distutils import sysconfig; print(os.path.realpath(os.path.join(sysconfig.get_config_var("LIBPL"), sysconfig.get_config_var("LDLIBRARY"))))' )"
        export PYTHON3_INCLUDE_DIR="$( ${PYTHON3_EXEC} -c 'from distutils import sysconfig; print(sysconfig.get_config_var("INCLUDEPY"))' )"
        export ANDROID_ABI=arm64-v8a
        export ANDROID_NATIVE_API_LEVEL=android-28
        export ANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang

1. Download the [Android SDK](https://developer.android.com/studio/#downloads) and [Android NDK](https://developer.android.com/ndk/downloads/index.html). This can be done with Android Studio or via the provided links. For Linux: If done via Android Studio the SDK will be already be in the right path given in compile.sh and the NDK can be found in "$HOME/Android/Sdk/ndk/". Set the environment variables `ANDROID_HOME` and `ANDROID_NDK` in compile.sh respectively.

1. Build (skipping packages that we don't need or can't cross-compile):

        ./compile.sh

You can find more information about the Android examples at https://github.com/ros2-java/ros2_android_examples

### Using ROS2 with android

You compiled libraries files with the .so file ending for the 64-bit ARM aarch64 architecture. You can find these files with the command
 find install/ -name "*.so"
and you can investigate their target architecture with the file command. E.g.
 file install/rmw_dds_common/lib/librmw_dds_common__rosidl_typesupport_c.so
Furthermore compiling yielded .jar files. These contain information like function names and signatures and serve the role of .h files in C.

To use the libraries, copy all you .jar files from the install folder to the app/libs folder of you android project (maybe create the folder). Add the line
 implementation fileTree(dir: 'libs', include: ['*.jar'])
to the apps build.gradle file in the dependencies section. Furthermore put all aarch64 .so files into app/src/main/jniLibs/arm64-v8a of your android project (maybe create the folder).

To investigate how to use the functions, use the jar tf command on the .jar files. For example running
 jar tf rcljava.jar
will yield a long list of classes, e.g. org/ros2/rcljava/node/BaseComposableNode.class
Replace the slashes with dots and omit the "class" appendix to get the import path:
 import org.ros2.rcljava.node.BaseComposableNode
In android studio CTRL+LMB on "BaseComposableNode" will lead to the respective .jar file, which provides more information.

Now you should be able to compile and run your app. Notice that the .so libraries are loaded at run time and therefore problems with them will also take place at run time.

### Custom message interfaces with android

Since the message interfaces are specific for the architecture and programming language of the target platform, care must be taken not only to compile for x86 with C h. headers, but also for aarch64 and .jar files.

Create the message definition in a respective ROS2 package as you would do it for C/C++/python. Copy this package folder to the workspace that you used to compile ROS2 for the 64-bit ARM aarch64 architecture. In compile.sh change the --packages-up-to option to also include your new package. Then execute compile.sh to compile for aarch64. Make sure that you did not source your x86 ROS2 environment in the terminal. If you encounter problems, it may be a good idea to completely remove the build and the install folder. When done, copy the .jar and .so files to your android project as described in the previous section. It is recommended to investigate the created .jar with "jar tf" to find the correct import path.

Here some example lines of java code (Kotlin will look slightly different):
import ros2_robotzoo_interfaces.msg.AprilTagWithCode;

public Publisher<ros2_robotzoo_interfaces.msg.AprilTagWithCode> publisher;

this.publisher = this.node.<ros2_robotzoo_interfaces.msg.AprilTagWithCode>createPublisher(
        ros2_robotzoo_interfaces.msg.AprilTagWithCode.class, "apriltag");

ros2_robotzoo_interfaces.msg.AprilTagWithCode msg = new ros2_robotzoo_interfaces.msg.AprilTagWithCode();
msg.setCode(898 + this.count);
msg.setCodeInvalid(123);
msg.setCodeUncertain(2323);
this.count++;
this.publisher.publish(msg);

Here setCodeInvalid() is a function automatically created as member of AprilTagWithCode. In android studion use CTRL+LMB on your message class to see such functions.

"this.node" denotes BaseComposableNode, since these lines of code were taken from a method of a  class, that inherits from BaseComposableNode.

If you want to use the message interface on a x86 system, you need to compile it another time - this time for x86. Use another workspace folder (not the android one), source your x86 ROS2 installation and compile with colcon as usual. Only after you sourced the setup-script in the install folder, you can work with the new interface definition, e.g. with ros2 topic echo.

## Contributing

Contributions are more than welcome!
If you'd like to contribute to the project, please read [CONTRIBUTING](CONTRIBUTING.md) for contributing guidelines.
