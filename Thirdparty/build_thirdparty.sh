#!/bin/bash

set +e # don't stop even error

NUM_JOBS=$(nproc)

mkdir -p Thirdparty_install
mkdir -p Thirdparty_build
cd Thirdparty_build

##################

echo ""
echo "Building Sophus lib!"
echo ""

mkdir -p Sophus
cd Sophus
cmake ../../Sophus/ -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="../../Thirdparty_install/"
make -j $NUM_JOBS install
cd ../


##################

echo ""
echo "Building Ceres lib!"
echo ""

mkdir -p ceres-solver
cd ceres-solver
cmake ../../ceres-solver/ -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DCMAKE_CXX_FLAGS="-march=native" -DCMAKE_INSTALL_PREFIX="../../Thirdparty_install/" -DBUILD_EXAMPLES=OFF
make -j $NUM_JOBS install
cd ../


##################

echo ""
echo "Building g2o lib!"
echo ""

mkdir -p g2o
cd g2o
cmake ../../g2o/ -DCMAKE_CXX_FLAGS="-march=native" -DCMAKE_INSTALL_PREFIX="../../Thirdparty_install/"
make -j $NUM_JOBS install
cd ../

##################

echo ""
echo "Building Pangolin lib!"
echo ""

cd ../Pangolin
# I don't know why it failed, but let's just ignore it for now
./scripts/install_prerequisites.sh recommended

cd ../Thirdparty_build
mkdir -p Pangolin
cd Pangolin

cmake ../../Pangolin/ -DCMAKE_INSTALL_PREFIX="../../Thirdparty_install/"
make -j $NUM_JOBS install
cd ../

