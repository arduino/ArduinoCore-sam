# Use with MSYS prompt
# export ARM_GCC_TOOLCHAIN=$LOCALAPPDATA/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/
export ARM_GCC_TOOLCHAIN=$LOCALAPPDATA/arduino15/packages/RFduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/

# build libsam
cd ../../system/libsam/build_gcc/
mingw32-make motionbow

# test build variant
cd ../../../variants/motionbow/build_gcc/
mingw32-make motionbow