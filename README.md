# 설치 및 실행 방법

1. Docker, Nvidia Container Toolkit & Isaac sim 설치
https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_container.html

2. Docker 이미지 build
```
docker build -f DOCKERFILE -t lg-vision:v1.0
```

3. Docker 컨테이너 실행
```
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -e DISPLAY \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    lg-vision:v1.0
```

4. Moveit2 Docker 설치
