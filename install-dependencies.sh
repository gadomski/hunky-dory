#!/usr/bin/env sh

set -e

repos=$HOME/Repos
local=$HOME/local
(cd $repos/fgt && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$local -G Ninja && ninja install)
(cd $repos/cpd && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$local -G Ninja && ninja install)
(cd $repos/laszip && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$local -G Ninja && ninja install)
(cd $repos/laz-perf && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$local -G Ninja && ninja install)
(cd $repos/PDAL && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$local -DWITH_LAZPERF=ON -G Ninja && ninja install)
(cd $repos/entwine && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$local -G Ninja && ninja install)
