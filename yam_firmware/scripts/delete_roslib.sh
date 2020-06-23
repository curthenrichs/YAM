#!/usr/bin/env bash

## Removes the ros_lib directory from the arduino sketchbook libraries directory
## Parameters
##  $1 = path to sketchbook libraries directory

cd $1
echo pwd
rm -rf ./ros_lib
