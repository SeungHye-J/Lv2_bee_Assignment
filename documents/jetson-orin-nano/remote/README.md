# 원격 처리 (SSH 키 · Linux 데스크톱)

`SETTING.md`에서 분리한 Jetson 원격 접속 문서다. SSH 키 등록부터 x11vnc, GNOME RDP, NoMachine까지 정리한다.

[← SETTING.md](../SETTING.md)

Jetson의 전체 Ubuntu 데스크톱을 원격에서 보려면 SSH 키 등록과 원격 데스크톱 설정을
분리해서 진행한다. SSH 터미널만 필요하면 키 등록까지만 하면 된다.

현재 예시 환경:

```text
PC       : 10.2.12.77
Jetson   : bee.local (10.2.12.110)
Jetson 사용자: bee
```

IP는 DHCP로 바뀔 수 있으므로 가능하면 `bee.local` 또는 고정 IP를 사용한다.

## 한눈에 보기

| 구분 | Jetson (화면을 공유하는 쪽) | PC (화면을 보는 쪽) |
| --- | --- | --- |
| 역할 | 실제 Ubuntu 데스크톱을 가진 서버 | 원격 화면을 보는 클라이언트 |
| 필요한 것 | `x11vnc`, 로그인된 데스크톱 | SSH 클라이언트, Remmina + VNC 플러그인 |
| 실행하는 것 | `x11vnc` | SSH 터널, Remmina |
| 주소 | `127.0.0.1:5900` | `localhost:5900` |

핵심은 간단하다.

```text
Jetson: Ubuntu 데스크톱 로그인 → x11vnc가 :0 화면을 공유
PC    : SSH 터널로 Jetson의 5900에 연결 → Remmina VNC로 화면 표시
```

### 설정 순서

1. **Jetson**: 자동 로그인 또는 모니터에서 `bee`로 그래픽 로그인
2. **Jetson**: `x11vnc` 설치 및 VNC 비밀번호 생성
3. **Jetson**: `x11vnc` 실행 (`127.0.0.1:5900`)
4. **PC**: SSH 터널 열기
5. **PC**: Remmina VNC → `localhost:5900`

## 빠른 실행 (스크립트)

같은 폴더에 두 스크립트가 있다.

```text
server.sh   # Jetson에서 실행 — x11vnc 시작/중지/상태
client.sh   # PC에서 실행 — SSH 터널 + Remmina
```

### Jetson: server.sh

Jetson에 스크립트를 복사해서 실행한다.

```bash
scp documents/jetson-orin-nano/remote/server.sh bee@bee.local:~/server.sh
ssh bee@bee.local 'chmod +x ~/server.sh && ~/server.sh'
```

이미 Jetson에 저장소가 있으면 그 경로에서 실행해도 된다.

```bash
cd ~/Lv2_bee_Assignment/documents/jetson-orin-nano/remote
./server.sh
```

기본 동작:

```text
주소      : 127.0.0.1:5900
display   : :0
비밀번호  : ~/.vnc/passwd
백그라운드 실행, 로그 ~/.vnc/x11vnc.log
```

자주 쓰는 명령:

```bash
./server.sh --status
./server.sh --stop
./server.sh --restart
./server.sh --scale 1280x720
./server.sh --foreground
```

`server.sh`는 항상 `-localhost`로 실행한다. PC에서 직접 `10.2.12.110:5900`으로
붙는 방식이 아니므로 아래 `client.sh`의 SSH 터널이 필요하다.

### PC: client.sh

PC에서 실행한다.

```bash
cd documents/jetson-orin-nano/remote
./client.sh
```

기본 동작:

```text
SSH 대상   : bee@bee.local
PC 포트    : localhost:5900
Jetson VNC : 127.0.0.1:5900
```

SSH 키가 이미 등록되어 있어야 한다. 접속 대상이 다르면:

```bash
./client.sh --host bee@10.2.12.110
```

PC의 5900 포트가 이미 사용 중이면:

```bash
./client.sh --local-port 5901
```

터널만 열고 VNC 클라이언트는 직접 실행하려면:

```bash
./client.sh --no-vnc
```

상태/종료:

```bash
./client.sh --status
./client.sh --stop
```

VNC 비밀번호는 Jetson 로그인 비밀번호가 아니라 `x11vnc -storepasswd`에서 만든
비밀번호다.

## 꼭 구분할 것

- `x11vnc`는 **Jetson에서만** 실행한다. PC에서 실행하는 프로그램이 아니다.
- SSH 터널과 Remmina는 **PC에서만** 실행한다.
- `-localhost`를 쓰면 Jetson의 5900은 외부에 열리지 않는다. 그래서 PC에서 SSH 터널이 필요하다.
- VNC 비밀번호는 `x11vnc -storepasswd`에서 만든 비밀번호다. Jetson 로그인 비밀번호와 다르다.

## 1. PC: SSH 키 등록

PC에 키가 없으면 먼저 만든다.

```bash
ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519 -C "bee@jetson"
```

Jetson에 공개키를 등록한다. 이때 Jetson 계정 비밀번호를 한 번 입력한다.

```bash
ssh-copy-id -i ~/.ssh/id_ed25519.pub bee@bee.local
```

확인:

```bash
ssh -o BatchMode=yes bee@bee.local 'hostname; hostname -I'
```

`Permission denied (publickey,password)`가 나오면 키 등록이 안 된 것이다.
`ssh-copy-id`를 다시 실행하거나 Jetson의 `~/.ssh/authorized_keys`를 확인한다.

매번 긴 주소를 입력하지 않으려면 PC의 `~/.ssh/config`에 추가한다.

```sshconfig
Host bee
  HostName bee.local
  User bee
  IdentitiesOnly yes
  IdentityFile ~/.ssh/id_ed25519
```

이제 다음처럼 접속한다.

```bash
ssh bee
```

## 2. SSH 접속과 operator_tool

`operator_tool`은 TUI이므로 GUI 없이 SSH만으로 동작한다.

```bash
ssh bee@bee.local

cd ~/Lv2_bee_Assignment/lv2_module6/tool
./run-cli
```

RViz 같은 개별 GUI 앱만 필요하면 X11 포워딩을 사용할 수 있다.

```bash
ssh -X bee@bee.local
cd ~/Lv2_bee_Assignment/lv2_module6/tool
./run-rviz
```

X11 포워딩은 전체 데스크톱이 아니라 앱 하나를 PC 화면에 띄우는 방식이다.
RealSense 영상이나 OpenGL 화면은 전달이 느릴 수 있다.

## 3. Jetson: x11vnc로 화면 공유

Jetson에 모니터가 연결되어 있고, 실제로 로그인한 GNOME 화면을 그대로 공유할 때
사용한다. **Jetson 모니터에서 `bee`로 로그인한 상태여야 한다.** GDM 로그인 화면은
일반 사용자 권한으로 공유하지 않는다.

Jetson에 `x11vnc`를 설치한다.

```bash
sudo apt update
sudo apt install -y x11vnc
```

Jetson 모니터의 데스크톱 터미널에서 데스크톱이 X11인지 확인한다.

```bash
echo "DISPLAY=$DISPLAY"
echo "XDG_SESSION_TYPE=$XDG_SESSION_TYPE"
```

`DISPLAY=:0`, `XDG_SESSION_TYPE=x11`이면 아래 명령을 그대로 쓸 수 있다.
Wayland 세션이면 x11vnc 대신 NoMachine이나 GNOME RDP를 사용한다.

VNC 비밀번호 파일을 만든다. `~/.vnc/passwd`가 생성된다.

```bash
mkdir -p ~/.vnc
x11vnc -storepasswd
```

Jetson에서 x11vnc를 실행한다.

```bash
x11vnc -display :0 -auth guess -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost
```

- `-display :0` : 실제 GNOME 데스크톱
- `-auth guess` : Xauthority 자동 탐색
- `-forever` : 클라이언트가 끊겨도 서버 유지
- `-shared` : 여러 클라이언트 동시 접속 허용
- `-localhost` : 로컬에서만 listen. 원격은 SSH 터널로 접속
- `-rfbauth` : VNC 비밀번호 파일

`-auth guess`가 실패하면 실제 Xauthority 경로를 지정한다.

```bash
# 예: 사용자 세션의 Xauthority
x11vnc -display :0 -auth "$HOME/.Xauthority" -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost
```

Jetson에서 5900 포트가 열렸는지 확인한다.

```bash
ss -ltnp | grep 5900
```

### PC: SSH 터널 + VNC 접속

PC에서 SSH 터널을 연다. 이 터미널은 접속 중 계속 열어 둔다.

```bash
ssh -N -L 5900:127.0.0.1:5900 bee@bee.local
```

이 명령의 의미:

```text
PC의 localhost:5900
  → SSH 터널
  → Jetson의 127.0.0.1:5900 (x11vnc)
```

PC에 VNC 클라이언트를 설치한다.

```bash
sudo apt install -y remmina remmina-plugin-vnc
# 또는
sudo apt install -y tigervnc-viewer
```

#### Remmina 명령어로 바로 접속

SSH 터널이 열려 있는 상태에서:

```bash
remmina -c vnc://localhost:5900
```

`vnc://`를 반드시 붙인다. `rdp://`가 아니고, 웹 브라우저 주소창에 넣는 것도 아니다.

터널을 백그라운드로 열고 Remmina까지 한 번에 실행하려면:

```bash
ssh -f -N -L 5900:127.0.0.1:5900 bee@bee.local
remmina -c vnc://localhost:5900
```

이미 5900 터널이 떠 있으면 두 번째 줄만 실행한다.
포트가 이미 사용 중이면 터널 포트를 5901로 바꾸고:

```bash
ssh -N -L 5901:127.0.0.1:5900 bee@bee.local
remmina -c vnc://localhost:5901
```

VNC 클라이언트에서 직접 주소를 입력할 때는:

```text
localhost:5900
```

`x11vnc -storepasswd`에서 설정한 VNC 비밀번호를 입력하면 Jetson의 전체 데스크톱이
나온다.

종료는 x11vnc를 실행한 Jetson 터미널에서 `Ctrl-C`, 또는:

```bash
pkill x11vnc
```

### 성능: 해상도 낮추기

1920x1080 화면을 32-bit로 보내면 VNC가 느려지기 쉽다. Jetson에서 실제 해상도를
1280x720으로 낮추면 가장 효과가 크다.

Jetson SSH에서:

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# 현재 연결된 출력 이름 확인 (예: DP-0)
xrandr --current

# 1280x720으로 변경
xrandr --output DP-0 --mode 1280x720
```

출력 이름은 `xrandr --current`의 `connected primary` 줄에서 확인한다.
예시에서는 `DP-0`을 사용했다.

해상도를 바꾼 뒤 x11vnc를 재시작한다.

```bash
pkill x11vnc

x11vnc -display :0 -auth guess -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost \
  -wait 20 -defer 40
```

다시 1920x1080으로 되돌리려면:

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
xrandr --output DP-0 --mode 1920x1080
```

해상도 자체를 바꾸지 않고 VNC 서버에서만 축소하려면 `-scale`을 사용한다.

```bash
x11vnc -display :0 -auth guess -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost \
  -scale 1280x720 -wait 20 -defer 40
```

Remmina 쪽에서도 연결 편집 → **고급 설정**에서:

```text
색상 깊이(Color depth): 8-bit 또는 16-bit
품질(Quality): Low/Poor
Tight encoding: 켬
```

로 맞추면 전송량이 크게 줄어든다.
`colordepth=32`는 가능하면 피한다.

> `-localhost`를 빼고 `0.0.0.0`에 열면 Wi-Fi/LAN의 다른 PC가 바로 접속할 수 있다.
> 보안상 권장하지 않으며, 외부 공유기 포트 포워딩은 절대 하지 않는다.
> 원격지에서 써야 하면 SSH 터널이나 Tailscale/WireGuard를 사용한다.

### x11vnc 자동 시작 (선택)

로그인 후 자동으로 x11vnc를 띄우려면 systemd 사용자 서비스를 만든다.
먼저 `x11vnc -storepasswd`로 `~/.vnc/passwd`를 만들어 둔다.

```bash
mkdir -p ~/.config/systemd/user

cat > ~/.config/systemd/user/x11vnc.service <<'EOF'
[Unit]
Description=x11vnc desktop sharing
After=graphical-session.target

[Service]
ExecStart=/usr/bin/x11vnc -display :0 -auth guess -forever -shared -rfbauth %h/.vnc/passwd -localhost
Restart=on-failure
RestartSec=2

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable --now x11vnc.service
systemctl --user status x11vnc.service
```

자동 시작을 끄려면:

```bash
systemctl --user disable --now x11vnc.service
```

## 4. Jetson: 키보드/모니터 없이 GDM 로그인

x11vnc는 `bee`가 GNOME 데스크톱에 **이미 로그인한 뒤에만** 사용할 수 있다.
GDM 로그인 화면은 `gdm` 사용자 권한으로 실행되므로, SSH에서 `bee`로 x11vnc를
띄우면 보통 다음처럼 실패한다.

```text
Authorization required, but no authorization protocol specified
xset: unable to open display ":0"
```

키보드 없이 Jetson 데스크톱에 로그인해야 하면 아래 두 방법 중 하나를 쓴다.

### 4.1 RDP Remote Login (권장)

Jetson의 GDM 로그인 화면 자체를 RDP로 보는 방법이다. `sudo` 권한이 필요하다.
`grdctl --system`은 TTY가 없으면 polkit 인증 오류가 날 수 있으므로 `ssh -t`를 쓴다.

```bash
ssh -t bee@bee.local \
  'sudo grdctl --system rdp enable && sudo grdctl --system status'
```

RDP가 3389 포트를 열었는지 확인한다.

```bash
ssh bee@bee.local 'ss -ltnp | grep 3389'
```

포트가 안 열리면 서비스를 재시작하고 로그를 확인한다.

```bash
ssh -t bee@bee.local \
  'sudo systemctl restart gnome-remote-desktop.service && sudo journalctl -u gnome-remote-desktop -n 50 --no-pager'
```

PC에서 RDP 클라이언트를 설치한다.

```bash
sudo apt install -y remmina remmina-plugin-rdp
```

Jetson의 3389 포트를 외부에 직접 열지 않으려면 PC에서 SSH 터널을 연다.

```bash
ssh -N -L 3389:localhost:3389 bee@bee.local
```

Remmina에서 접속한다.

```text
프로토콜: RDP - 원격 데스크톱 프로토콜
서버: localhost:3389
사용자: bee
비밀번호: Jetson 계정 비밀번호
```

GDM 로그인 화면이 나오면 `bee`로 로그인한다. 이제 키보드 없이 전체 Ubuntu
데스크톱을 사용할 수 있다.

SSH 터널 없이 같은 LAN에서 직접 붙을 때는 서버를 `10.2.12.110:3389`로 지정한다.
다만 RDP 포트를 외부 인터넷에 노출하지 않는다.

RDP가 검은 화면이거나 GNOME/Xorg 조합에서 불안정하면 아래 자동 로그인 또는
NoMachine을 사용한다.

### 4.2 임시 GDM 자동 로그인 (RDP가 안 될 때)

Jetson이 부팅될 때 `bee`로 자동 로그인하게 만든다. 보안상 **임시방편**으로만
사용하고, 원격 접속이 확인되면 자동 로그인은 끄는 것을 권장한다.

```bash
ssh -t bee@bee.local 'sudo nano /etc/gdm3/custom.conf'
```

`[daemon]` 아래에 추가한다.

```ini
[daemon]
AutomaticLoginEnable=True
AutomaticLogin=bee
```

저장한 뒤 GDM을 재시작한다.

```bash
ssh -t bee@bee.local 'sudo systemctl restart gdm'
```

잠시 기다린 뒤 `bee`의 그래픽 세션이 생겼는지 확인한다.

```bash
ssh bee@bee.local 'loginctl list-sessions; ps -eo pid,user,args | grep "[X]org"'
```

`seat0` 세션이 `active`이고 Xorg 프로세스가 보이면 x11vnc를 실행할 수 있다.
`DISPLAY=:0`은 Jetson 모니터의 데스크톱 터미널에서 확인한다.

```bash
x11vnc -display :0 -auth guess -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost
```

PC에서:

```bash
ssh -N -L 5900:localhost:5900 bee@bee.local
```

Remmina VNC로 `localhost:5900`에 접속한다.

자동 로그인을 끄려면 `custom.conf`에서 두 줄을 제거하거나 주석 처리한 뒤:

```bash
ssh -t bee@bee.local 'sudo systemctl restart gdm'
```

> 자동 로그인은 Jetson에 물리적으로 접근할 수 있는 사람에게 데스크톱을 바로
> 넘긴다. 공용 로봇이나 외부 네트워크에 연결된 장비에서는 오래 켜 두지 않는다.

## 5. NoMachine 대안 (Jetson + PC)

NoMachine은 Jetson ARM64용 `.deb`를 제공하며, 전체 데스크톱을 비교적 쾌적하게
사용할 수 있다.

```text
nomachine.com → ARM64 .deb 다운로드
```

Jetson에서:

```bash
sudo apt install ./nomachine_*_arm64.deb
sudo systemctl enable --now nxserver
systemctl status nxserver --no-pager
```

PC에 NoMachine 클라이언트를 설치하고 접속한다.

```text
호스트: 10.2.12.110
포트: 4000
사용자: bee
```

## 6. 원격 GUI와 ROS 주의사항

- `operator_tool`은 SSH 터미널에서 실행하는 것이 가장 가볍다.
- RViz를 Jetson에서 띄우고 x11vnc/RDP/NoMachine으로 화면만 보는 방식이
  Wi-Fi에서 RealSense raw 영상을 직접 전송하지 않아 유리하다.
- PC에서 RViz가 직접 ROS 토픽을 구독하게 하려면 `ROS_LOCALHOST_ONLY`를 끄고
  `ROS_DOMAIN_ID=88`을 맞춰야 한다. 이 경우 카메라/점구름 트래픽이 Wi-Fi를 탄다.
- Jetson 로컬에서만 ROS를 쓰려면 `ROS_LOCALHOST_ONLY=1`을 유지해도 되고,
  원격 데스크톱으로 화면만 보면 된다.

## 7. 원격 접속 체크리스트

```text
[ ] ssh-copy-id로 공개키 등록
[ ] ssh bee@bee.local 접속 성공
[ ] (모니터 있음) Jetson 모니터에서 bee 그래픽 로그인
[ ] (모니터 없음) RDP remote login 활성화 → GDM 원격 로그인
[ ] (모니터 없음) ss -ltnp | grep 3389 확인
[ ] (모니터 없음) Remmina RDP → localhost:3389
[ ] (RDP 불가) AutomaticLogin + gdm 재시작
[ ] Jetson 데스크톱 터미널에서 echo $DISPLAY → :0, XDG_SESSION_TYPE=x11
[ ] x11vnc -storepasswd로 ~/.vnc/passwd 생성
[ ] x11vnc ... -localhost 실행
[ ] ss -ltnp | grep 5900 확인
[ ] PC에서 ssh -N -L 5900:127.0.0.1:5900 bee@bee.local
[ ] remmina -c vnc://localhost:5900
[ ] (느리면) Jetson 1280x720 + Remmina 8-bit 색상
```

문제가 생기면 먼저 확인할 것:

```bash
# SSH 키
ssh -v bee@bee.local

# RDP remote login
ss -ltnp | grep 3389
systemctl status gnome-remote-desktop --no-pager

# x11vnc 로그/포트
ss -ltnp | grep 5900
ps -ef | grep '[x]11vnc'

# 데스크톱 세션
echo "$DISPLAY $XDG_SESSION_TYPE"
loginctl list-sessions
```
