#!/usr/bin/env bash

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
    mkdir -p build
    cd build
    cmake ..
    cd ..
fi

# Build project.
cd build
make -j
cd ..

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output
#bin/PA1 testcases/cornell.txt output/cornell.bmp
bin/PA1 testcases/testfocal.txt output/focal.bmp
#bin/PA1 testcases/testmesh.txt output/testmesh.bmp
#bin/PA1 testcases/testpic.txt output/testpic.bmp
#bin/PA1 testcases/testtexture.txt output/testtexture.bmp
#bin/PA1 testcases/testoval.txt output/testoval.bmp
bin/PA1 testcases/easter.txt output/easter.bmp
#bin/PA1 testcases/norm.txt output/testbezier.bmp
bin/PA1 testcases/china.txt output/china.bmp