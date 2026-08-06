#!/bin/bash
set -e

echo "Quick setup"

if ! command -v docker &>/dev/null; then
    echo "ERROR: Docker not installed"
    exit 1
fi

if [ ! -d "external/eigen" ]; then
    echo "Eigen is not installed. Cloning now.."
    mkdir -p external
    git clone --branch 3.4.0 https://gitlab.com/libeigen/eigen.git external/eigen
# else
#     echo "Eigen is installed"
fi

echo "Building docker image"
docker build -t aria_env_img .

echo "Setup good. Run `sh run.sh /path/to/vrs/file` in your terminal"
