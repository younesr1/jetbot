#!/bin/sh
# Allow usage of docker commands without sudo
sudo groupadd docker
sudo usermod -aG docker $USER
# build the docker image
sudo docker build -t jetbot-docker -f ../Dockerfile .


