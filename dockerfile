FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update && apt-get dist-upgrade -y

RUN apt-get install -y g++ git cmake libprotobuf-dev \
      protobuf-compiler php php-cli php-xml \
      libgtest-dev libqt5widgets5 qt5-default libqt5webkit5-dev libwebsockets-dev \
      python-pip python-empy

RUN pip install -U catkin_tools


RUN mkdir -p /ssl/src/{rhoban,rhobandeps}

WORKDIR /ssl/src/rhoban
RUN git clone https://github.com/Rhoban/geometry.git
RUN git clone https://github.com/Rhoban/random.git
RUN git clone https://github.com/Rhoban/utils.git

WORKDIR /ssl/src/rhobandeps
RUN git clone https://github.com/RhobanDeps/eigen.git
RUN git clone https://github.com/RhobanDeps/serial.git
RUN git clone https://github.com/RhobanDeps/jsoncpp.git
RUN git clone https://github.com/RhobanDeps/tclap.git


WORKDIR /ssl

COPY . .

RUN ./workspace setup

RUN ./workspace install

RUN ./workspace build

CMD ./bin/ai_st -s -y 