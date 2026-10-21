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