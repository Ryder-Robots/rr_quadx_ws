#! /bin/bash

function fail() {
   msg=${1}
   echo "ERROR: unable to install: ${msg}"
   exit 1
}

# Wrapper for installing packages
function aptinstall(){
    pkg=${1}
    sudo apt-get install -y  ${pkg}
    if [[ $? -ne 0 ]]; then
        echo "ERROR: unable to install package: ${pkg}"
        exit 1
    fi
}