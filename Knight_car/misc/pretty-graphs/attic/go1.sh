#!/bin/bash
set -e
set -x

bag="$1"
vehicle="$2"

echo "bag: $bag" 
echo "vehicle: $vehicle" 

mkdir -p out/
out="$bag.camera.mp4"

topic="/$vehicle/camera_node/image/compressed"
echo "topic $topic"
echo "out $out"

export DISABLE_CONTRACTS=1 

pg -m procgraph_ros bag2mp4 --bag $bag --topic $topic --out $out

