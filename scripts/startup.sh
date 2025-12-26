#!/bin/bash

# Specify the PATH
export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin"
export PYTHONPATH=$PYTHONPATH:/home/intech/CoVAPSy


# Change to the project directory
cd /home/intech/CoVAPSy

# Pull the latest changes from the repository
git pull 

uv sync --extra rpi
uv run /home/intech/CoVAPSy/src/HL/Serveur_mq.py
