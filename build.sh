#!/bin/bash

rm -rf build
mkdir build
cd build

clear

cmake ../
make

cd ..
cp ./build/SPI_LCD_DEMO ~/Public/.
