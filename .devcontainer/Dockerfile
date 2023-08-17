FROM mcr.microsoft.com/devcontainers/base:jammy

# RUN apk update && apk add
RUN apt-get update && apt-get install -y libeigen3-dev \
                                        libsdl2-dev \
                                        libsdl2-ttf-dev \
                                        cmake \
                                        x11-apps \
                                        dbus-x11 \
                                        openssh-server \
                                        openssh-client \
                                        net-tools \
                                        valgrind
RUN echo "debug"
RUN dbus-uuidgen > /etc/machine-id