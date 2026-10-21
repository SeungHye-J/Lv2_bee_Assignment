# OS
1. sdk 실행
2. 리커버리 모드진입
3. 플러그인 선택 x
4. [download](https://developer.nvidia.com/sdk-manager)
5. runtime, ssd 선택
6. ubuntu 유저 세팅 skip
7. display 연결
8. ubuntu 설정 (유저 이름, 비번 설정)
9. sudo usermod -aG dialout,video bee
10. reboot
11. ssh 연결 확인 
- ssh user@jetson.local





# ros setting
## link
- https://nvidia-isaac-ros.github.io/getting_started/isaac_ros_buildfarm_cdn.html







*ROS 2 Lyrical 설치 가이드 (Ubuntu 24.04 / Jetson JetPack 7.2 공통)*
ROS 공식 저장소에는 arm64용 Lyrical이 없어서 *NVIDIA Isaac ROS 저장소*에서 설치함. 팀 노트북도 같은 방식임.

*1. 기본 도구 · universe*
```
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
```

*2. ROS 공식 저장소 등록 (ros2-apt-source)*
```
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

*3. Isaac ROS 저장소 등록 (Lyrical은 여기서 받음)*
```
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-isaac-ros.gpg
echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-5.0 noble main" | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list
echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/ubuntu/main noble main" | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros-buildfarm.list
sudo apt update
apt-cache policy ros-lyrical-ros-base
```
• 마지막 줄에서 Candidate에 버전이 나와야 함

*4. ROS 2 · 빌드 도구 설치*
```
sudo apt install -y ros-lyrical-ros-base python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pip build-essential cmake
sudo rosdep init
rosdep update
```
• `ros-lyrical-desktop-full`은 설치하지 말 것 (우리 `perception` 패키지와 이름 겹침)

*5. 환경 변수 (~/.bashrc)*
```
echo -e '\n# ROS 2 Lyrical\nsource /opt/ros/lyrical/setup.bash\nexport ROS_DOMAIN_ID=88' >> ~/.bashrc
source ~/.bashrc
```
• `ROS_DOMAIN_ID=88` 팀 공통값, 꼭 맞출 것
• 부트캠프 Wi-Fi는 멀티캐스트가 막혀서 자동 탐색이 안 됨 → `ROS_STATIC_PEERS`에 상대 IP 필수. 여러 대면 `"IP1;IP2"`
• 이름(`xxx.local`) 대신 *IP로* 적을 것. 이름은 상대가 꺼져 있으면 노드 실행 자체가 실패함
• 내 IP 확인: `hostname -I`
• ~/.bashrc 고친 뒤엔 열려 있는 터미널마다 `source ~/.bashrc` (안 하면 예전 설정으로 실행됨)

*6. 확인*
```
echo $ROS_DISTRO $ROS_DOMAIN_ID $ROS_STATIC_PEERS
ros2 doctor --report | grep -E "distribution name|middleware name"
```
• `lyrical 17 <IP>` / `lyrical`, `rmw_fastrtps_cpp` 나오면 정상

*7. 통신 테스트*
기기 A
```
ros2 topic pub -r 2 /test std_msgs/msg/String "{data: hello}"
```
기기 B
```
ros2 topic echo --once /test std_msgs/msg/String
```
• `data: hello` 나오면 성공. 안 되면 양쪽 IP · ROS_STATIC_PEERS · DOMAIN_ID 확인 (Wi-Fi IP는 바뀔 수 있음)






- ros2 development tool
```bash
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep
```
- ros library
```
sudo apt-get install -y --no-install-recommends \
    ros-dev-tools \
    python3-colcon-common-extensions \
    ros-lyrical-nav2-msgs \
    ros-lyrical-moveit-msgs \
    ros-lyrical-moveit-task-constructor-msgs \
    ros-lyrical-control-msgs \
    ros-lyrical-controller-interface \
    ros-lyrical-hardware-interface \
    ros-lyrical-pluginlib \
    ros-lyrical-std-srvs \
    ros-lyrical-controller-manager \
    ros-lyrical-joint-state-broadcaster \
    ros-lyrical-imu-sensor-broadcaster \
    ros-lyrical-diff-drive-controller \
    ros-lyrical-joint-trajectory-controller \
    ros-lyrical-parallel-gripper-controller \
    ros-lyrical-robot-state-publisher \
    ros-lyrical-xacro \
    ros-lyrical-ros2controlcli \
    ros-lyrical-twist-mux \
    ros-lyrical-diagnostic-updater \
    ros-lyrical-diagnostic-aggregator \
    ros-lyrical-teleop-twist-keyboard \
    ros-lyrical-robot-localization \
    ros-lyrical-rosx-introspection \
    ros-lyrical-cv-bridge \
    libopencv-dev \
    rapidjson-dev \
    socat \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*
```


```bash
sudo apt-get install -y --no-install-recommends \
    ros-lyrical-nav2-lifecycle-manager \
    ros-lyrical-nav2-bt-navigator \
    ros-lyrical-nav2-behavior-tree \
    ros-lyrical-nav2-planner \
    ros-lyrical-nav2-navfn-planner \
    ros-lyrical-nav2-controller \
    ros-lyrical-nav2-dwb-controller \
    ros-lyrical-nav2-behaviors \
    ros-lyrical-nav2-costmap-2d \
    ros-lyrical-nav2-velocity-smoother \
    ros-lyrical-nav2-collision-monitor \
    ros-lyrical-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*
```

```bash
sudo apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-moveit-ros-move-group \
    ros-lyrical-moveit-kinematics \
    ros-lyrical-moveit-planners-ompl \
    ros-lyrical-moveit-simple-controller-manager \
    ros-lyrical-moveit-ros-visualization \
    && rm -rf /var/lib/apt/lists/*
```

```bash
sudo apt-get install -y --no-install-recommends \
    git ros-lyrical-rsl ros-lyrical-generate-parameter-library ros-lyrical-moveit-core ros-lyrical-tf2-kdl \
    librange-v3-dev libfmt-dev libexpected-dev \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /tmp/pick_ik_ws/src && cd /tmp/pick_ik_ws/src \
    && git clone https://github.com/PickNikRobotics/pick_ik.git && cd pick_ik && git checkout 7c97fe2 \
    && cd /tmp/pick_ik_ws && . /opt/ros/lyrical/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && cp -r install/pick_ik/. /opt/ros/lyrical/ \
    && rm -rf /tmp/pick_ik_ws
```


```bash

sudo apt-get install -y --no-install-recommends \
    ros-lyrical-rtabmap-msgs \
    ros-lyrical-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*
```

```bash
# realsense
sudo apt update
sudo apt install ros-$ROS_DISTRO-realsense2-camera
```


```bash
sudo apt-get install -y --no-install-recommends \
    ros-lyrical-rtabmap-msgs \
    ros-lyrical-rtabmap-slam \
    ros-lyrical-rtabmap-odom \
    ros-lyrical-rtabmap-util \
    ros-lyrical-rtabmap-sync \
    ros-lyrical-rtabmap-launch
```

- rtapmap error 우선 설치해보고 안되면  이것ㄷ 안되면 ros-lyrical-rtabmap-map 만깔아라 
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone -b lyrical-devel https://github.com/introlab/rtabmap.git src/rtabmap
git clone -b lyrical-devel https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j2"
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```



```bash
sudo apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 python3-pip ros-lyrical-ament-cmake-pytest \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --break-system-packages --ignore-installed --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu \
    && pip install --break-system-packages --ignore-installed --no-cache-dir ultralytics pytest

```


# 1. Arduino CLI 설치
```bash
mkdir -p "$HOME/bin"
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
| BINDIR="$HOME/bin" sh

export PATH="$HOME/bin:$PATH"
arduino-cli version

# PATH 영구 등록
grep -q 'export PATH="$HOME/bin:$PATH"' ~/.bashrc || \
echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
# 2. OpenCR 설정값
export OPENCR_VERSION=1.5.3
export OPENCR_SHA256=418656e5e6d99d45d187ffdb28dece0f450c6707da3f6db56769f3ecafdc413c

CLI="$HOME/bin/arduino-cli"
FQBN="OpenCR:OpenCR:OpenCR"
# 3. Arduino CLI 설정 — sudo 금지
"$CLI" config init --overwrite
"$CLI" config add board_manager.additional_urls \
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
"$CLI" core update-index
# 4. 시스템 컴파일러 (여기만 sudo)
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
sudo rm -rf /var/lib/apt/lists/*
# 5. Jetson arm64용 OpenCR 코어 수동 설치
#    ROBOTIS 기본 툴체인은 x86용이라 시스템 arm-none-eabi를 링크해서 씁니다.
curl -fsSL \
"https://github.com/ROBOTIS-GIT/OpenCR/releases/download/${OPENCR_VERSION}/opencr.tar.bz2" \
-o /tmp/opencr.tar.bz2

echo "${OPENCR_SHA256}  /tmp/opencr.tar.bz2" | sha256sum -c -

mkdir -p "$HOME/.arduino15/packages/OpenCR/hardware/OpenCR/${OPENCR_VERSION}" \
"$HOME/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin"

rm -rf /tmp/opencr
tar xzf /tmp/opencr.tar.bz2 -C /tmp
cp -a /tmp/opencr/. "$HOME/.arduino15/packages/OpenCR/hardware/OpenCR/${OPENCR_VERSION}/"

rm -rf /tmp/opencr /tmp/opencr.tar.bz2

for f in /usr/bin/arm-none-eabi-*; do
ln -sf "$f" "$HOME/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/"
done
# 6. Dynamixel 라이브러리
"$CLI" lib install Dynamixel2Arduino

# 7. 확인
"$CLI" board listall | grep -i opencr
```
```text
정상이면:
OpenCR Board    OpenCR:OpenCR:OpenCR
가 보입니다.  
수동 설치라 core list에는 안 보일 수 있는데, board listall에 OpenCR이 보이면 컴파일은 됩니다.
Jetson에서는 업로드가 안 됩니다
OpenCR 업로더 opencr_ld는 x86_64용만 제공됩니다.  
그래서 Jetson에서는:
컴파일   ✅
업로드   ❌
입니다.
Jetson에서 컴파일만:
"$CLI" compile --fqbn "$FQBN" \
/home/bee/.../lv2_module6/firmware/beebot_opencr
.bin 파일은 여기에 생깁니다.
find "$HOME/.cache/arduino/sketches" -name '*.ino.bin'
이 .bin을 OpenCR이 연결된 x86 PC로 복사해서 업로드하거나,  
업로드는 지금처럼 x86 PC에서 Arduino CLI로 하는 것이 안전합니다.
정리
Jetson에서 Arduino CLI 설치       ✅
OpenCR 코어/컴파일                 ✅ (시스템 gcc 사용)
Jetson에서 OpenCR 업로드          ❌ (opencr_ld arm64 없음)
업로드                            x86 PC에서
sudo arduino-cli는 절대 쓰지 말고, $HOME/bin/arduino-cli를 일반 사용자로 실행하세요.
```


```
# CI 배포 패키지 (Ubuntu 24.04 / Jetson)

CI가 `aarch64`용을 두 가지로 만든다.

| 패키지 | 대상 |
| --- | --- |
| `ros2_ws-deploy-aarch64.tar.gz` | Ubuntu 26.04 ARM |
| `ros2_ws-deploy-aarch64-noble.tar.gz` | **Ubuntu 24.04 / Jetson Orin Nano** |

Jetson에서는 RUN ID로 받으면 `/etc/os-release`와 `uname -m`을 보고 noble 패키지를 자동 선택한다.

```bash
cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
./download-artifact.sh <RUN ID>
```

패키지에는 `TARGET` 파일이 들어 있고, `start.sh`가 장치 OS와 다르면 실행을 막는다.

```bash
cat TARGET
# ubuntu-24.04-aarch64

./start.sh
```

> Ubuntu 24.04용 패키지는 ROS 2 Lyrical이 `/opt/ros/lyrical`에 설치되어 있어야 한다.
> 위 “ROS 2 Lyrical 설치 가이드”를 먼저 따른다.






# gz error
```bash

sudo apt update
sudo apt install ros-$ROS_DISTRO-gz-gui
# 또는 Gazebo 시뮬레이션 관련 메타 패키지 전체 설치
sudo apt install ros-$ROS_DISTRO-ros-gz



sudo apt update
sudo apt install libgz-gui-dev  # 혹은 버전에 따라 libgz-gui8-dev 등
```



# opencr 고정하기
```bash
lsusb
udevadm info -a -n /dev/ttyACM0
```
## editor
```
sudo vi /etc/udev/rules.d/99-opencr.rules
```
내용[99-opencr.rules](../../lv2_module6/ros2_ws/99-opencr.rules): SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="opencr", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"

