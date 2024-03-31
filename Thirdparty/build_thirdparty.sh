#!/bin/bash

set +e # don't stop even error

mkdir -p Thirdparty_install

##################

echo ""
echo "Building Sophus lib!"
echo ""

cd Sophus

mkdir -p build
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="../../Thirdparty_install/"
make -j8 install
cd ../../


##################

echo ""
echo "Building Ceres lib!"
echo ""

cd ceres-solver
mkdir -p build
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DCMAKE_CXX_FLAGS="-march=native" -DCMAKE_INSTALL_PREFIX="../../Thirdparty_install/" -DBUILD_EXAMPLES=OFF
make -j8 install
cd ../../


##################

echo ""
echo "Building g2o lib!"
echo ""

cd g2o
mkdir -p build
cd build/
cmake .. -DCMAKE_INSTALL_PREFIX="../../Thirdparty_install/"
make -j8 install
cd ../../

##################

echo ""
echo "Building Pangolin lib!"
echo ""

cd Pangolin

# I don't know why it failed, but let's just ignore it for now
./scripts/install_prerequisites.sh recommended

mkdir -p build
cd build/
cmake .. -DCMAKE_INSTALL_PREFIX="../../Thirdparty_install/"
make -j8 install
cd ../../

