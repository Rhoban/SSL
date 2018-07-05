FROM ubuntu:16.04

RUN apt-get update && \
    apt-get install -y php php-xml python-pip python-empy cmake git nano graphviz doxygen libprotobuf-dev protobuf-compiler libqt5widgets5 qt5-default libqt5webkit5-dev && \
    apt-get autoclean -y && \
    pip install catkin_tools

ARG SSH_KEY
ARG USER=pfe
ARG USER_ID=1000
ARG USER_GID=1000

RUN groupadd --gid "${USER_GID}" "${USER}" && \
    useradd \
      --uid ${USER_ID} \
      --gid ${USER_GID} \
      --create-home \
      ${USER}

RUN php -r "copy('https://github.com/google/googletest/archive/release-1.8.0.tar.gz', '/tmp/gtest.tar.gz');" && \
    mkdir /tmp/googletest && \
    tar --strip-components=1 -C /tmp/googletest/ -xzvf /tmp/gtest.tar.gz && \
    rm /tmp/gtest.tar.gz && \
    cd /tmp/googletest && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr CMakeLists.txt && \
    make && make install && \
    rm -rf /tmp/googletest

COPY entrypoint.sh /bin/entrypoint.sh

# Patch catkin_tools: exclude prebuilt packages from build jobs when --no-deps is present
RUN sed -i 's/jobs = \[\] + list(prebuild_jobs.values())/jobs = \[\] + (list(prebuild_jobs.values()) if not no_deps else \[\])/g' \
	/usr/local/lib/python2.7/dist-packages/catkin_tools/verbs/catkin_build/build.py

USER ${USER}

RUN mkdir -p ~/.ssh && \
    ssh-keyscan -H bitbucket.org github.com > ~/.ssh/known_hosts

ENTRYPOINT [ "/bin/entrypoint.sh" ]
VOLUME [ "/workspace" ]
WORKDIR /workspace
