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

RUN python3 -m pip install pytest
WORKDIR /home/pilot

# Install MissionPlanner
USER root
RUN apt-get install -y mono-devel
# RUN mkdir /MissionPlanner
# ADD https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip /MissionPlanner.zip
# RUN unzip -d /MissionPlanner /MissionPlanner.zip \
#     && rm /MissionPlanner.zip

# Create a virtual in-memory X server and a recorder
RUN apt-get install -y xvfb ffmpeg
ENV DISPLAY=:99
COPY scripts/virtual_xserver/record_virtual_Xserver.sh /home/pilot/screen_record/record_virtual_Xserver.sh
COPY scripts/virtual_xserver/start_and_record_virtual_Xserver.sh /home/pilot/screen_record/start_and_record_virtual_Xserver.sh

WORKDIR /home/pilot/app
ENTRYPOINT [ "Xvfb", ":99", "-screen", "0", "1920x1080x24" ]
# CMD "Xvfb :99 -screen 0 1920x1080x24"
# CMD "Xvfb :99 -screen 0 1920x1080x24 &; /bin/bash /home/pilot/screen_record/record_virtual_Xserver.sh &; /bin/bash"
# CMD "Xvfb :99 -screen 0 1920x1080x24 &; /bin/bash /home/pilot/screen_record/record_virtual_Xserver.sh &; python3 -m pytest ."
# CMD "python3 -m pytest ."
