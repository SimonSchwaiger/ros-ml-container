#!/bin/bash
rm .plaidml

mkdir src
# git clone ros packages here

mkdir app
# put runnable scripts in here

echo "{
    "PLAIDML_DEVICE_IDS":[
        "llvm_cpu.0"
    ],
    "PLAIDML_EXPERIMENTAL":false
}" >> .plaidml

docker build -t fhtw3dof .

