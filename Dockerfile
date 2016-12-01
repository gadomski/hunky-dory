FROM connormanning/entwine:latest
MAINTAINER Howard Butler <howard@hobu.co>

ENV CC gcc
ENV CXX g++

RUN apt-get update && apt-get install -y --fix-missing --no-install-recommends \
        libgdal1-dev \
        libboost1.58-all-dev \
        libflann-dev \
    && rm -rf /var/lib/apt/lists/*


RUN git clone https://github.com/gadomski/cpd.git \
    && cd cpd \
    && git checkout master \
    && cmake . -DWITH_TESTS=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release \
    && make \
    && make install \
    && rm -rf /cpd

RUN git clone http://github.com/gadomski/hunky-dory.git \
    && cd hunky-dory \
    && git checkout master \
    && git submodule init && git submodule update \
    && mkdir build \
    && cd build \
    && cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DFLANN_INCLUDE_DIRS="/usr/include;/usr/include/gdal;/usr/include/jsoncpp" \
        .. \
    && make -j4 \
    && make install
