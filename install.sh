#!/bin/sh

rm -rf External

mkdir External && cd External
git clone https://github.com/giaf/blasfeo.git
git clone https://github.com/giaf/hpipm.git
git clone https://github.com/acados/acados.git

cd blasfeo
mkdir build
cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$(realpath ../)
make -j$(nproc) && make install
cd ../../

cd hpipm
mkdir build
cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$(realpath ../) -DBLASFEO_PATH=$(realpath ../../blasfeo)
make -j$(nproc) && make install
cd ../../

cd acados
git submodule update --recursive --init
mkdir build
cd build && cmake -DACADOS_WITH_QPOASES=ON ..
make -j$(nproc) && make install
cd ../external
wget -q -nc --show-progress https://github.com/casadi/casadi/releases/download/3.5.0/casadi-linux-matlabR2014b-v3.5.0.tar.gz
mkdir -p casadi-matlab
tar -xf casadi-linux-matlabR2014b-v3.5.0.tar.gz -C casadi-matlab
cd ../../
