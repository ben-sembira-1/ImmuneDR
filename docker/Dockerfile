FROM ubuntu:18.04

RUN mkdir /Drones
RUN apt-get update \
    && apt-get install -y python3.8 python3.8-dev python3-pip \
    && cd /usr/local/bin \
    && ln -s /usr/bin/python3 python \
    && pip3 install --upgrade pip \
    && cd /
RUN apt-get install -y git
RUN git clone https://github.com/ArduPilot/ardupilot.git /ardupilot 
RUN python3 -m pip install pymavlink

RUN cd /ardupilot \
    && git submodule update --init --recursive
RUN apt-get install -y sudo
# RUN useradd -ms /bin/bash user_not_root
# RUN echo "user_not_root ALL=(ALL) NOPASSWD:/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh" >> /etc/sudoers
# USER user_not_root
# RUN cd /ardupilot \
#     && Tools/environment_install/install-prereqs-ubuntu.sh -y
USER root
RUN python3 -m pip install empy pexpect
RUN cd /ardupilot \
    && ./waf configure --board sitl \
    && ./waf copter

RUN sudo apt-get install -y python3-dev
# RUN sudo apt-get install -y python3-opencv
RUN sudo apt-get install -y python3-wxgtk4.0
RUN sudo apt-get install -y python3-pip
RUN sudo apt-get install -y python3-matplotlib
RUN sudo apt-get install -y python3-lxml
RUN pip3 insatll pygame
RUN pip3 install PyYAML mavproxy --user
RUN echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
