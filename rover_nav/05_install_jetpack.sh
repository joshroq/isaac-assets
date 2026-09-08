#!/bin/bash
set +e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_sudo.sh"
echo "START jetpack install $(date)"
rsudo apt-get update -y
# CUDA runtime + VPI are what cuVSLAM/nvblox dlopen (libcudart.so.12, libnvvpi.so.3).
# nvidia-jetpack is the complete, no-guess set (CUDA, cuDNN, TensorRT, VPI, multimedia).
rsudo apt-get install -y nvidia-jetpack
echo "jetpack RC=$?"
# make CUDA discoverable at runtime
if [ -d /usr/local/cuda/lib64 ]; then
  echo "/usr/local/cuda/lib64" | rsudo tee /etc/ld.so.conf.d/cuda.conf
  rsudo ldconfig
fi
echo "libcudart: $(ldconfig -p | grep -c cudart)  libnvvpi: $(ldconfig -p | grep -c nvvpi)"
echo "DONE jetpack $(date)"; touch ~/rover_nav/logs/jetpack.done
