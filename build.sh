#!/bin/sh

#  build.sh
#  ExtendedKF
#
#  Created by Ollie Steiner on 17/06/17.
#

mkdir build
cd build
cmake ..
make
./ExtendedKF
