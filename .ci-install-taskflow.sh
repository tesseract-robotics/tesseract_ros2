set -e
TASKFLOW_VERSION="v3.3.0"
git clone https://github.com/taskflow/taskflow.git /tmp/taskflow
git -C /tmp/taskflow checkout $TASKFLOW_VERSION
mkdir /tmp/taskflow-build
cmake -S /tmp/taskflow -B /tmp/taskflow-build \
  -DCMAKE_INSTALL_PREFIX=/usr \
  -DTF_BUILD_BENCHMARKS=OFF \
  -DTF_BUILD_CUDA=OFF \
  -DTF_BUILD_EXAMPLES=OFF \
  -DTF_BUILD_TESTS=OFF
sudo cmake --install /tmp/taskflow-build/
rm -rf /tmp/taskflow/ /tmp/taskflow-build/
