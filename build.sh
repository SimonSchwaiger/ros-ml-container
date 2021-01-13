#!/bin/bash

#mkdir src
# git clone ros packages here

#mkdir app
# put runnable scripts in here

rm .plaidml

#echo "{
#    "PLAIDML_DEVICE_IDS":[
#        "llvm_cpu.0"
#    ],
#    "PLAIDML_EXPERIMENTAL":false
#}" >> .plaidml


# enable plaidml gpu support in container
echo "{
    "PLAIDML_DEVICE_IDS":[
        "opencl_amd_gfx1010.0"
    ],
    "PLAIDML_EXPERIMENTAL":true
} " >> .plaidml


docker build -t fhtw3dof .

