#!/bin/bash

apt install python3-full
python3 -m venv /home/pi/python
source /home/pi/python/bin/activate
pip install --upgrade pip
pip install pyserial RPi.GPIO
