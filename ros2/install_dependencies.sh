# 만약을 위해 만들어본 파일, 아마 사용하지 않을 것 같음

#!/bin/bash

# Ubuntu/Debian 시스템 의존성 설치
sudo apt-get update
sudo apt-get install -y nlohmann-json3-dev libmosquitto-dev

# Python 의존성 설치
pip install -r requirements.txt 