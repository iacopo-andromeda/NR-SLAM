# If the build directory already exists, remove it
if [ -d "build" ]; then
    rm -rf build
fi
mkdir build
cd build

cmake ..

make -j8