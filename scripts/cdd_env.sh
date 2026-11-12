# Source before launching TensorFlow so glibc sees the pip CUDA library paths.
CDD_PYTHON="${CDD_PYTHON:-$ROOT/.venv-cdd-wsl/bin/python}"
CDD_CUDA_LIBS="$("$CDD_PYTHON" -c 'import glob,sys; print(":".join(glob.glob(sys.prefix+"/lib/python*/site-packages/nvidia/*/lib")))')"
export LD_LIBRARY_PATH="$CDD_CUDA_LIBS:/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TF_CPP_MIN_LOG_LEVEL=1
