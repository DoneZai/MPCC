#!/bin/sh

mkdir external && cd external
git clone https://github.com/giaf/blasfeo.git
git clone https://github.com/giaf/hpipm.git

cd blasfeo
mkdir build && mkdir lib
cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$(realpath ../)
make -j$(nproc) && make install
cd ../../

cd hpipm
mkdir build && mkdir lib
cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$(realpath ../) -DBLASFEO_PATH=$(realpath ../../blasfeo)
make -j$(nproc) && make install
cd ../../


