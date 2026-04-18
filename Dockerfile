FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    cmake g++ make git \
    libssl-dev libjpeg-dev libbluetooth-dev \
  && rm -rf /var/lib/apt/lists/*
WORKDIR /sunray
