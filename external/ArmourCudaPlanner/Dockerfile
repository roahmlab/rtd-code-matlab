ARG CUDASRC=/tmp/armour
ARG RUNPATH=/armour
ARG IPOPTVER=3.14.10
ARG IPOPTINSTALL=/opt/ipopt
ARG HSLINSTALL=/opt/hsl
ARG HSLFILE=coinhsl.tar.gz

# Create the execution image
FROM nvidia/cuda:12.0.0-runtime-ubuntu22.04 AS runtime

# Settings for apt
ENV DEBIAN_FROTNEND="noninteractive" TZ="Etc/UTC"

# =========== Basic Configuration ======================================================
# Common apt requirements and python3
COPY apt-common.txt apt-common.txt
RUN apt-get update \
    && apt-get install --no-install-recommends -y `grep -ve \# apt-common.txt | xargs` \
    && apt-get install --no-install-recommends -y python3 \
    && apt-get clean \
    && apt-get -y autoremove \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf apt-common.txt

# Build image
FROM nvidia/cuda:12.0.0-devel-ubuntu22.04 AS builder

# =========== Basic Configuration ======================================================
# Common apt requirements and those required to build IPOPT
COPY apt-common.txt apt-common.txt
RUN apt-get update \
    && apt-get install --no-install-recommends -y `grep -ve \# apt-common.txt | xargs` \
    && apt-get install --no-install-recommends -y \
       build-essential gcc g++ gfortran git patch wget \
       pkg-config liblapack-dev libblas-dev libmetis-dev \
       apt-utils \
    && apt-get clean \
    && apt-get -y autoremove \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf apt-common.txt

# ======== Start IPOPT installation ====================================================
# Retrieve and copy all the dependencies needed by Ipopt
ARG IPOPTDIR=/tmp/ipopt
ARG IPOPTVER
ARG IPOPTINSTALL
ARG HSLINSTALL
ARG HSLFILE

WORKDIR /tmp
RUN wget https://github.com/coin-or/Ipopt/archive/refs/tags/releases/${IPOPTVER}.tar.gz
RUN tar xvf ./${IPOPTVER}.tar.gz && mv Ipopt-releases-${IPOPTVER} ${IPOPTDIR}

# Prerequisites for Ipopt
# HSL
WORKDIR ${IPOPTDIR}
RUN git clone --depth 1 --branch releases/2.2.1 https://github.com/coin-or-tools/ThirdParty-HSL.git
WORKDIR ${IPOPTDIR}/ThirdParty-HSL
COPY ${HSLFILE} ${HSLFILE}
RUN tar xvf ./${HSLFILE} && mv $(ls -d coinhsl*/) coinhsl
RUN ./configure --prefix=${HSLINSTALL} \
    && make \
    && make install
RUN find ${HSLINSTALL}/lib/libcoinhsl.* | xargs -I{} echo {} {} | sed "s/coin//2" | xargs -n 2 ln -s
RUN ln -s ${HSLINSTALL}/lib/pkgconfig/coinhsl.pc ${HSLINSTALL}/lib/pkgconfig/hsl.pc

# Configure and compile Ipopt
WORKDIR ${IPOPTDIR}
RUN mkdir build
WORKDIR ${IPOPTDIR}/build
RUN ../configure --prefix=${IPOPTINSTALL} \
    && make \
    && make install

# Update environment variables
ENV IPOPT_HOME=${IPOPTINSTALL}
ENV LD_LIBRARY_PATH=${IPOPT_HOME}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV PKG_CONFIG_PATH=${IPOPT_HOME}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}
ENV C_INCLUDE_PATH=${IPOPT_HOME}/include/coin-or${C_INCLUDE_PATH:+:${C_INCLUDE_PATH}}
ENV HSL_HOME=${HSLINSTALL}
ENV LD_LIBRARY_PATH=${HSL_HOME}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV PKG_CONFIG_PATH=${HSL_HOME}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}
ENV C_INCLUDE_PATH=${HSL_HOME}/include/coin-or${C_INCLUDE_PATH:+:${C_INCLUDE_PATH}}

#============= Do the CUDA Planner Build =======================================

# Required to build code
RUN apt-get -y update \
    && apt-get install --no-install-recommends -y \
       libboost-dev \
    && apt-get clean \
    && apt-get -y autoremove \
    && rm -rf /var/lib/apt/lists/*

# Build (refresh the arg)
ARG CUDASRC
ARG RUNPATH

WORKDIR ${CUDASRC}
# MUST USE EIGEN 3.3.7
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz \
    && tar -xvf eigen-3.3.7.tar.gz \
    && mv eigen-3.3.7 eigen3
COPY src/* ./
#RUN ./compile.sh
#RUN nvcc -o armour_main armour_main.cu NLPclass.cu CollisionChecking.cu Dynamics.cu Trajectory.cu PZsparse.cu -Xcompiler -fopenmp -std=c++14 -O2 -I/usr/local/include -I/opt/ipopt/include/coin-or -L/usr/local/lib -L/usr/lib -L/opt/ipopt/lib -lipopt -lquadmath -lstdc++ -ldl -lm -lc -lgomp
RUN nvcc -o armour_main armour_main.cu NLPclass.cu CollisionChecking.cu Dynamics.cu Trajectory.cu PZsparse.cu -Xcompiler -fopenmp -std=c++14 -O2 -I./ -I/usr/local/include -I/opt/ipopt/include/coin-or -L/usr/local/lib -L/usr/lib -L/opt/ipopt/lib -lipopt -lquadmath -lstdc++ -ldl -lm -lc -lgomp

# prepare to export
RUN tar -cvzf transfer.tar.gz ${IPOPT_HOME} ${HSL_HOME} armour_main

#=============== Return to the runtime container and prepare application =================
FROM runtime

# import these args
ARG CUDASRC
ARG RUNPATH
ARG IPOPTINSTALL
ARG HSLINSTALL

# Set Environment variables
ENV IPOPT_HOME=${IPOPTINSTALL}
ENV LD_LIBRARY_PATH=${IPOPT_HOME}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV PKG_CONFIG_PATH=${IPOPT_HOME}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}
ENV C_INCLUDE_PATH=${IPOPT_HOME}/include/coin-or${C_INCLUDE_PATH:+:${C_INCLUDE_PATH}}
ENV HSL_HOME=${HSLINSTALL}
ENV LD_LIBRARY_PATH=${HSL_HOME}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV PKG_CONFIG_PATH=${HSL_HOME}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}
ENV C_INCLUDE_PATH=${HSL_HOME}/include/coin-or${C_INCLUDE_PATH:+:${C_INCLUDE_PATH}}


WORKDIR ${RUNPATH}
# Copy built files
COPY --from=builder ${CUDASRC}/transfer.tar.gz transfer.tar.gz
RUN tar -xvf transfer.tar.gz -C / \
   && mv /armour_main armour

# Copy server
COPY server.py server.py

# Application
EXPOSE 65535/tcp
CMD ["python3", "server.py"]
