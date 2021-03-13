#!/bin/bash
git submodule init
git submodule sync
git submodule update
make clean
make
./paparazzi