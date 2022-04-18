FROM ubuntu:20.04

ENV TZ=Europe/Kiev
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update -y && \
    apt-get install -y tzdata  && \
    apt-get install -y git python g++ ccache gawk make wget cmake python3-pip iproute2 python3-future libtool autoconf pkg-config ninja-build sudo


RUN pip3 install meson==0.56

RUN adduser --disabled-password --gecos '' pilot
RUN adduser pilot sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER pilot

WORKDIR /home/pilot

# Install mavlink router - will enable connecting with external ground station for debugging
RUN git clone --recursive --depth 1 https://github.com/mavlink-router/mavlink-router.git
WORKDIR /home/pilot/mavlink-router
RUN meson setup build . -Dsystemdsystemunitdir=/usr/lib/systemd/system --buildtype=debugoptimized
USER root
RUN ninja -C build install
USER pilot


# Install ardupilot
WORKDIR /home/pilot

RUN git clone --recursive --depth 1 https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/pilot/ardupilot
RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections && \
    sudo apt-get install -y -q keyboard-configuration

RUN USER=pilot Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN python3 ./waf configure && python3 ./waf copter

WORKDIR /home/pilot

RUN pip3 install pytest

WORKDIR /home/pilot/app
CMD "python3 -m pytest ."