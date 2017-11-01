#!/bin/sh
sudo servod --p1pins=3,5,7,11,13,15,19,21,23,29,31,33 \
--min=10us --max=5000us \
--step-size=2us --cycle-time=5000us