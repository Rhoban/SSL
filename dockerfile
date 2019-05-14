FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive 

RUN mkdir /ssl

WORKDIR /ssl

RUN apt-get update && apt-get dist-upgrade -y

RUN apt-get install -y g++ git cmake libprotobuf-dev \
      protobuf-compiler php php-cli php-xml \
      libgtest-dev libqt5widgets5 qt5-default libqt5webkit5-dev libwebsockets-dev \
      python-pip python-empy

RUN pip install -U catkin_tools

COPY . .

RUN ./workspace setup

RUN ./workspace install

RUN ./workspace build:debug

EXPOSE 7882

CMD ./bin/ai_st -s -y 