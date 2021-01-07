#!/bin/bash
rm .plaidml

echo "{
    "PLAIDML_DEVICE_IDS":[
        "llvm_cpu.0"
    ],
    "PLAIDML_EXPERIMENTAL":false
}" >> .plaidml

docker build -t fhtw3dof .

