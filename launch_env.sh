#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="11.8"
fi

export STAGING_ROOT="/data/safe_staging"

# TOP require flask

if ! python3 -c "import flask" &> /dev/null; then
  echo "Flask is not installed, installing..."
  sudo apt update
  sudo apt install python3-pip
  sudo apt install python3-venv
  sudo -H python3 -m pip install flask
  read -n 1 -s
else
  echo "Flask is already installed"
  echo -en "1" > /data/params/d/SecondBoot
fi
