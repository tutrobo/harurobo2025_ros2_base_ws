```bash
docker build --platform linux/amd64,linux/arm64 \
  -t eyr1n/harurobo2025-ros2-base-ws .
docker image push eyr1n/harurobo2025-ros2-base-ws
```

```bash
sudo apt install podman containers-storage
sudo podman pull docker.io/eyr1n/harurobo2025-ros2-base-ws
sudo podman run --privileged --rm -it --net=host --ipc=host \
  -v /dev/serial0:/dev/serial0 \
  docker.io/eyr1n/harurobo2025-ros2-base-ws
```

```bash
sudo podman run --privileged --rm -it --net=host --ipc=host \
  -v /dev/serial0:/dev/serial0 \
  docker.io/eyr1n/harurobo2025-ros2-base-ws bash
```
