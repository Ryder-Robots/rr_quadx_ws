#! /bin/bash

function fail() {
   msg=${1}
   echo "ERROR: unable to install: ${msg}"
   exit 1
}