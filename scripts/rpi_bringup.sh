#!/bin/bash

# Call MicroXRCEAgent to set up the serial connection with Pixhawk
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 115200

# Call launch files
