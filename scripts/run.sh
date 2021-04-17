#!/bin/sh
# remove existing containers
docker rm /master
# open a bash terminal
docker run -it --name master jetbot-docker bash
