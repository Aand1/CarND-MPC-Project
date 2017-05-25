#!/bin/bash

wget -nc https://github.com/udacity/self-driving-car-sim/releases/download/v1.3/term2_sim_linux.zip
unzip term2_sim_linux.zip

mv term2_sim_linux/term2_sim.x86_64 ./simulator
mv term2_sim_linux/term2_sim_Data/ ./simulator_Data
rm -fR term2_sim_linux
chmod u+x simulator
