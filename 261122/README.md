# 3·4일차 (261121 ~ 261122) ROS2 기초: 토픽, 런치, 서비스, 사용자 정의 인터페이스

3일차에는 ROS2 Jazzy 개발 환경을 설정하고 노드, 토픽, 메시지, 서비스, 액션 같은 ROS2의 기본 개념과 CLI 명령어, QoS를 배웠다. 이어서 Colcon 워크스페이스에 첫 Python 패키지(`1_pubsub`)를 만들어 talker/listener 노드를 실행했고, 수업용 자율주행 로봇을 조립했다.
4일차에는 WASD 키보드 입력으로 로봇 위치를 계산하는 3개 노드(publisher, processor, subscriber)를 작성하고, 런치 파일로 여러 노드를 한 번에 실행했다. 또한 사용자 정의 메시지와 서비스(`custom_interfaces`)를 정의하고, 이를 사용하는 서비스 서버·클라이언트(`2_srvmsg`)로 두 수 더하기와 2차원 로봇 이동 계산을 실습했다.

---

## 1. 파일 구조

```
Robotics-202136021/
├── 261121/                                  # 3일차
│   └── 1_pubsub/                            # ament_python 패키지
│       ├── package.xml
│       ├── setup.py                         # talker, listener 실행 명령 등록
│       ├── setup.cfg
│       ├── resource/1_pubsub
│       ├── test/
│       └── 1_pubsub/
│           ├── talker.py                    # 1초마다 /demo_topic에 문자열 발행
│           └── listener.py                  # /demo_topic 구독 후 출력
└── 261122/                                  # 4일차
    ├── README.md                            # 3·4일차 정리 문서 (이 파일)
    ├── 1_pubsub/                            # 3일차 패키지에 4일차 실습을 추가
    │   ├── package.xml
    │   ├── setup.py                         # 실행 명령 5개 + 런치 파일 등록
    │   ├── launch/
    │   │   ├── talk_listen.launch.py        # talker, listener 동시 실행
    │   │   └── my_launch.launch.py          # publisher, processor, subscriber 동시 실행
    │   └── 1_pubsub/
    │       ├── talker.py, listener.py       # 3일차와 동일
    │       ├── publisher.py                 # WASD 키 입력을 /wasd에 발행
    │       ├── processor.py                 # /wasd 구독 → 위치 계산 → /location 발행
    │       └── subscriber.py                # /location 구독 후 출력
    ├── custom_interfaces/                   # ament_cmake 인터페이스 패키지
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── msg/
    │   │   ├── Num.msg                      # int64 my_num
    │   │   └── RobotPose.msg                # float64 x, y, theta
    │   └── srv/
    │       ├── AddTwoInts.srv               # Num a, Num b → Num sum
    │       └── MoveRobot.srv                # RobotPose, distance → RobotPose, success
    └── 2_srvmsg/                            # ament_python 패키지
        ├── package.xml                      # custom_interfaces 의존성
        ├── setup.py                         # 실행 명령 4개 등록
        └── 2_srvmsg/
            ├── add_ints_server.py           # 두 수를 더해 응답
            ├── add_ints_client.py           # 명령행 인자 두 개로 요청
            ├── move_robot_server.py         # 현재 자세와 이동 거리로 새 위치 계산
            └── move_robot_client.py         # x, y, theta, distance로 요청
```

- 각 날짜 폴더는 실습 당시 `~/robot_navigation/colcon_ws/src/` 안에 있던 패키지를 그대로 옮긴 것이다. `build/`, `install/`, `log/`는 빌드할 때 생성되는 폴더이므로 저장소에 포함하지 않았다.

---

## 2. 개발 환경 및 의존성

| 구분 | 내용 |
| --- | --- |
| 운영체제 | Ubuntu 24.04 |
| ROS2 | Jazzy |
| 언어 | Python 3 (rclpy), 인터페이스 패키지는 CMake (ament_cmake) |
| 빌드 도구 | colcon |
| 통신 미들웨어 | CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) |
| ROS_DOMAIN_ID | 11 (수업에서 부여받은 값) |
| ROS 패키지 | `rclpy`, `std_msgs`, `geometry_msgs`, `launch`, `rosidl_default_generators`, `rosidl_default_runtime` |
| 예제·도구 | `demo_nodes_cpp`, `turtlesim`, `rqt_graph` |
| Python 라이브러리 | `numpy` (`processor.py`에서 좌표 계산) |
| 기타 | `gnome-terminal` (런치 파일이 노드마다 새 터미널을 열 때 사용) |
| 하드웨어 | 수업용 자율주행 로봇 (ESP32 PCB, DC 모터 4개, 라이다, IMU) |

---

## 3. 실행 방법

### 3.1 ROS2 환경 설정

터미널을 열 때마다 필요한 설정을 `~/.bashrc`에 등록한다. `>>`는 파일 끝에 한 줄을 추가하는 것이므로 같은 명령을 여러 번 실행하면 중복된 줄이 쌓인다. 등록 전에 `grep -n ros ~/.bashrc`로 이미 있는지 확인한다.

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=11" >> ~/.bashrc
echo "source ~/robot_navigation/colcon_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

마지막 줄의 `install/setup.bash`는 워크스페이스를 한 번 빌드한 뒤에 생긴다.

### 3.2 워크스페이스 구성 및 빌드

```bash
git clone https://github.com/SungYeopLim/Robotics-202136021.git
mkdir -p ~/robot_navigation/colcon_ws/src
cp -r Robotics-202136021/261122/* ~/robot_navigation/colcon_ws/src/
rm ~/robot_navigation/colcon_ws/src/README.md

cd ~/robot_navigation/colcon_ws
colcon build
source install/setup.bash
```

- 빌드는 항상 워크스페이스 최상위(`colcon_ws`)에서 한다.
- `2_srvmsg`는 `custom_interfaces`에 의존하므로, colcon이 `custom_interfaces`를 먼저 빌드한다.
- 코드나 `setup.py`를 수정하면 다시 빌드하고 `source install/setup.bash`를 실행해야 바뀐 내용이 반영된다.
- 패키지 이름이 숫자로 시작한다는 경고가 나오지만 실행에는 문제가 없다.

### 3.3 [3일차] ROS2 예제 노드와 CLI

```bash
# 터미널 1
ros2 run demo_nodes_cpp talker
# 터미널 2
ros2 run demo_nodes_cpp listener
# 터미널 3
ros2 node list
ros2 topic list
ros2 topic echo /chatter
rqt_graph
```

turtlesim으로 토픽 발행과 서비스 목록을 확인한다.

```bash
ros2 run turtlesim turtlesim_node
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 service list
```

### 3.4 [3일차] talker / listener (`1_pubsub`)

```bash
# 터미널 1
ros2 run 1_pubsub talker
# 터미널 2
ros2 run 1_pubsub listener
```

### 3.5 [4일차] WASD 위치 계산 (`publisher` → `processor` → `subscriber`)

터미널 3개에서 각각 실행하거나, 런치 파일로 한 번에 실행한다.

```bash
# 각각 실행
ros2 run 1_pubsub publisher
ros2 run 1_pubsub processor
ros2 run 1_pubsub subscriber

# 런치 파일로 실행 (노드마다 새 터미널이 열린다)
ros2 launch 1_pubsub my_launch.launch.py
ros2 launch 1_pubsub talk_listen.launch.py   # talker, listener
```

publisher 터미널에서 `w`, `a`, `s`, `d` 중 하나를 입력하고 Enter를 누르면 subscriber 터미널에 이동 전후 좌표가 출력된다.

### 3.6 [4일차] 서비스 (`2_srvmsg`)

```bash
# 두 수 더하기
ros2 run 2_srvmsg add_ints_server
ros2 run 2_srvmsg add_ints_client 3 5

# 2차원 로봇 이동: x y theta(라디안) distance
ros2 run 2_srvmsg move_robot_server
ros2 run 2_srvmsg move_robot_client 0 0 0.5236 1.2
```

정의한 인터페이스 구조는 다음 명령으로 확인한다.

```bash
ros2 interface show custom_interfaces/srv/MoveRobot
```

---

## 4. 이론적 배경

### 4.1 3일차 (261121)

#### 4.1.1 ROS2의 기본 구조

ROS2는 여러 프로그램(노드)이 정해진 이름의 채널(토픽)로 메시지를 주고받으며 하나의 로봇 시스템을 이루는 구조이다. 라디오에 비유하면 다음과 같다.

| 용어 | 뜻 | 라디오 비유 |
| --- | --- | --- |
| node | 한 가지 역할을 하는 실행 단위 | 방송국 하나, 라디오 하나 |
| topic | 메시지가 오가는 채널의 이름 | 주파수(채널) |
| publisher | 토픽에 메시지를 보내는 쪽 | 방송(발행)한다 |
| subscriber | 토픽의 메시지를 받는 쪽 | 듣는다(구독) |
| message / msg type | 주고받는 데이터와 그 형식 | 방송 내용과 형식 |

- 실습에서는 `ros2 run` 한 번에 노드 하나가 실행되므로 노드 1개 = 프로세스 1개 = 터미널 1개로 생각하면 이해하기 쉽다. 실제 로봇 한 대 안에서는 센서, 판단, 주행 노드처럼 여러 노드가 동시에 동작한다.
- subscriber는 publisher가 사용하는 토픽 이름과 메시지 타입을 정확히 지정해야 메시지를 받을 수 있다.
- 토픽 이름은 `/`로 그룹화할 수 있다. 예: `/한국/경제`, `/미국/경제`, `/미국/날씨`, `/turtle1/cmd_vel`

#### 4.1.2 메시지 타입과 계층 구조

메시지는 필드마다 `int64`, `float64`, `string`처럼 타입이 정해져 있어 Python보다 C의 구조체에 가깝다. 또한 메시지 안에 다른 메시지가 들어 있는 계층 구조이므로, 값을 읽거나 쓸 때는 최종 변수 이름까지 점(`.`)으로 타고 들어간다. 편지 봉투 안의 봉투를 차례로 여는 것과 같다.

- `geometry_msgs/msg/Pose`: 로봇의 상태(state)는 내 위치와 내 방향 두 가지로 표현된다.
  - `position`: x, y, z
  - `orientation`: x, y, z, w (쿼터니언)
  - 예: `pose.position.x`
- `geometry_msgs/msg/Twist`: 속도 명령
  - `linear`: x, y, z (직선 속도)
  - `angular`: x, y, z (회전 속도)
  - 바퀴 로봇은 주로 `linear.x`(전진)와 `angular.z`(제자리 회전)를 사용한다.
- `geometry_msgs/msg/TwistStamped`: Twist에 시간 도장(`header.stamp`의 `sec`, `nanosec`)이 찍힌 메시지
  - 예: `msg.header.stamp.sec`, `msg.twist.linear.x`

로봇에 속도를 직접 명령하려면 메시지를 만들고 최종 변수(`linear.x` 등)를 하나씩 지정한 뒤 발행한다. 메시지를 계층화하면 `Vector3`, `Point` 같은 작은 타입을 여러 메시지가 재활용할 수 있어서, 데이터 타입이 무수히 늘어나는 비효율을 막을 수 있다. 메시지 타입과 구조는 ROS2 공식 문서나 `ros2 interface show <타입>`으로 확인한다.

#### 4.1.3 서비스와 액션

토픽은 한쪽이 계속 보내고 다른 쪽이 계속 듣는 방식이다. 이와 달리 요청에 대한 응답이 필요한 통신에는 서비스와 액션을 사용한다.

- **서비스**: 퀵서비스처럼 요청하면 바로 응답을 받는다. 예: 로봇에게 "위치가 어디야?"라고 물으면 로봇이 바로 답한다.
- **액션**: 오래 걸리는 요청이다. 예: 로봇에게 이동을 요청하면 로봇은 이동하면서 "10% 도착", "20% 도착" 같은 피드백을 계속 보내고, 끝나면 결과를 보낸다.

서비스도 토픽처럼 타입이 있으며, 요청(request)과 응답(response) 두 부분으로 나뉜다. 자세한 구조는 4일차(4.2.3 ~ 4.2.5)에서 다룬다.

#### 4.1.4 ROS2 CLI

ROS2 명령어는 `ros2 <대상> <동작> <인자>` 구조로 되어 있어, 구조를 알면 처음 보는 명령도 쉽게 사용할 수 있다.

| 명령 | 의미 |
| --- | --- |
| `ros2 run <패키지> <실행 파일>` | 노드 실행. 예: `demo_nodes_cpp`(패키지) + `talker`(실행 파일). 라이브러리 이름과 함수 이름을 지정하는 것과 비슷하다 |
| `ros2 node list` | 실행 중인 노드 목록 |
| `ros2 topic list` | 존재하는 토픽 목록 |
| `ros2 topic echo /<토픽>` | 토픽에 오가는 메시지 출력 |
| `rqt_graph` | 노드와 토픽의 연결 관계를 그래프로 시각화 |
| `ros2 topic pub /<토픽> <타입> "<값>"` | 터미널에서 직접 메시지 발행 |
| `ros2 service list` | 존재하는 서비스 목록 |

`ros2 topic pub`의 마지막 인자는 메시지의 최하단 변수 값을 YAML 형식으로 지정한 것이다. 메시지 타입은 Tab 키로 자동 완성된다.

| 옵션 | 의미 |
| --- | --- |
| `--once` | 한 번만 발행 |
| `-r 10` | 10Hz 주기(초당 10번)로 발행 |

`/cmd_vel`은 로봇의 속도 명령에 공통적으로 쓰이는 토픽 이름이다(turtlesim은 `/turtle1/cmd_vel`). `Twist`의 `linear`, `angular` 값은 모두 실수형이므로 `2.0`처럼 소수점을 넣어야 한다.

#### 4.1.5 QoS (Quality of Service)

노드끼리 데이터를 주고받을 때 적용하는 통신 규칙이다.

| 항목 | 선택지 | 의미 | 사용 예 |
| --- | --- | --- | --- |
| Reliability (신뢰성) | Reliable | 유실된 데이터를 다시 보내 하나도 빠짐없이 전달한다. 실시간성은 떨어질 수 있다 | 빠지면 안 되는 명령 |
| | Best Effort | 최선을 다해 보내지만 재전송하지 않는다(최고는 아님). 빠르지만 일부 유실을 허용한다 | 라이다 같은 센서 데이터 |
| Durability (지속성) | Volatile (기본값) | 휘발성. 보낸 메시지를 남기지 않는다 | 대부분의 토픽 |
| | Transient Local | 최근 메시지(History depth 개수만큼)를 일시적으로 내부에 저장했다가 늦게 연결된 구독자에게도 전달한다 | 한 번만 보내는 지도 데이터 |

등산객에게 지도를 나눠주는 상황에 비유하면, 지도를 다 나눠준 뒤 늦게 온 사람이 지도를 받지 못하는 것이 Volatile의 약점이고, 지도를 따로 보관해 두었다가 늦게 온 사람에게도 주는 것이 Transient Local이다.

코드의 `create_publisher(String, '/demo_topic', 10)`에서 `10`은 History depth(대기열에 보관할 메시지 수)이며, 나머지 QoS는 기본값(Reliable, Volatile)이 적용된다.

#### 4.1.6 Colcon 워크스페이스와 빌드

**Colcon 워크스페이스**는 ROS2 패키지의 개발과 관리를 위한 작업 공간(폴더)이다. 여러 패키지를 하나의 프로젝트 단위로 구성하고, 소스 코드와 빌드 결과물을 분리해서 관리한다.

| 디렉토리 | 역할 |
| --- | --- |
| `src/` | 개발자가 작성한 ROS2 패키지의 소스 코드. 패키지 하나가 폴더 하나이다 |
| `build/` | 컴파일 및 빌드 과정에서 생성되는 중간 파일(거쳐 가는 파일) |
| `install/` | 빌드 및 설치된 실행 파일, 라이브러리, 설정 파일. `ros2 run`은 이 폴더의 파일을 실행한다 |
| `log/` | 빌드 및 테스트 과정의 로그 |

**Colcon 빌드 도구**는 ROS2 패키지의 빌드와 테스트를 관리하는 명령줄 도구(CLI)이다.

- 워크스페이스 안의 패키지를 자동으로 찾는다.
- 패키지 간 의존성을 분석해 빌드 순서를 정한다.
- 여러 패키지를 한꺼번에, 또는 선택해서 빌드할 수 있다.

**`source install/setup.bash`가 필요한 이유**: 터미널은 열릴 때 패키지를 찾을 경로 목록(환경 변수)을 읽어 두고 그대로 사용한다. 빌드하면 `install/`에 새 실행 파일이 생기지만, 터미널은 아직 그 위치를 모른다. `source install/setup.bash`는 이 목록에 내 워크스페이스의 `install/`을 추가하는 명령이다. 이 줄을 `~/.bashrc`에 등록하면 새 터미널을 열 때마다 자동으로 적용된다.

**패키지 생성**: `ros2 pkg create` 명령의 패키지 이름 자리(`pkg_name`)에는 실제로 사용할 이름을 지정한다. `1_pubsub`처럼 숫자로 시작하는 이름은 Python에서 `import 1_pubsub`처럼 직접 쓸 수 없어 경고가 나오지만, `ros2 run`은 다른 방식으로 불러오므로 실행에는 문제가 없다.

#### 4.1.7 Python 패키지 구조와 `setup.py`

`1_pubsub` 패키지에서 알아야 할 것은 두 가지이다.

- 패키지와 같은 이름의 `1_pubsub/` 폴더: Python 소스 코드(`talker.py`, `listener.py`)를 넣는 곳
- `setup.py`: 패키지를 생성할 때 자동으로 만들어지는 등록증

`talker.py`와 `listener.py`는 Python이 아니라 ROS가 실행하므로, ROS가 알아볼 수 있게 `setup.py`에 등록해야 한다.

- **`entry_points`**: 터미널에서 사용할 실행 명령 이름을 등록한다. 형식은 `'명령 이름 = 패키지.파일이름:함수이름'`이고, 실행 함수는 `main()`이다. 리스트이므로 항목 사이의 쉼표(`,`)를 빠뜨리면 안 된다. 명령 이름과 파일 이름을 같게 맞추는 것이 관례이며, 개발자와 관리자 모두 관리하기 편하다.

  ```python
  'talker = 1_pubsub.talker:main',
  'listener = 1_pubsub.listener:main',
  ```

- **`data_files`**: 빌드할 때 어떤 파일을 `install/` 아래 어디로 복사할지 알려준다. 튜플들의 리스트이며, 각 튜플의 첫 번째 값은 복사될 위치, 두 번째 값은 복사할 파일 목록이다. 기본으로 있는 두 줄은 그대로 두고, 런치 파일처럼 폴더를 추가할 때 줄을 늘린다(4일차에서 사용).

#### 4.1.8 노드 코드의 구조

```python
class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, '/demo_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
```

| 코드 | 의미 |
| --- | --- |
| `class Talker(Node)` | `Node`는 rclpy가 제공하는 노드의 부모 클래스이다. 상속하면 `create_publisher`, `create_timer`, `get_logger` 등을 바로 쓸 수 있다 |
| `super().__init__('talker')` | 노드 이름을 `talker`로 정한다. `ros2 node list`에 보이는 이름이다 |
| `create_publisher(타입, 토픽, 10)` | 해당 토픽에 메시지를 보내는 publisher를 만든다 |
| `create_timer(1.0, 콜백)` | 1초 타이머가 만료될 때마다 콜백(`publish_message`)을 실행한다. 퍼블리싱 주기를 정하는 역할이다 |
| `msg = String()` → `msg.data = ...` | 메시지 객체를 만들고 최종 변수 `data`에 값을 넣는다 |
| `self.publisher_.publish(msg)` | 실제로 메시지를 보내는 줄이다 |
| `self.get_logger().info(...)` | 터미널에 로그를 출력한다. 발행과는 별개이다 |
| `create_subscription(타입, 토픽, 콜백, 10)` | 메시지가 올 때마다 콜백을 실행하는 subscriber를 만든다. 콜백은 받은 메시지를 인자로 받는다 |

`main()`의 흐름은 다음과 같다.

1. `rclpy.init()`: ROS2 통신을 시작한다.
2. `node = Talker()`: Talker 클래스로 노드 객체를 생성한다.
3. `rclpy.spin(node)`: 타이머 만료나 메시지 수신 같은 이벤트를 기다렸다가 콜백을 실행하는 무한 루프이다. Ctrl+C로 종료하면 루프를 빠져나온다.
4. `node.destroy_node()`, `rclpy.shutdown()`: 노드를 정리하고 통신을 종료한다.

#### 4.1.9 수업용 로봇 하드웨어와 조립

로봇의 센서와 모터는 모두 ESP32 PCB에 연결된다. ESP32가 저수준 제어(모터 구동, 센서 값 읽기)를 맡고, ROS2가 동작하는 컴퓨터가 고수준 판단을 맡는다.

| 부품 | ESP32 GPIO |
| --- | --- |
| 모터 FR (앞 오른쪽) | IN1 = 26, IN2 = 25 |
| 모터 FL (앞 왼쪽) | IN1 = 19, IN2 = 27 |
| 모터 RR (뒤 오른쪽) | IN1 = 23, IN2 = 12 |
| 모터 RL (뒤 왼쪽) | IN1 = 0, IN2 = 15 |
| IMU (MPU6050) | SDA = 21, SCL = 22 |
| 라이다 | RX = 16 |

- 모터 하나는 핀 2개로 방향을 제어한다(IN1 HIGH·IN2 LOW는 정회전, 반대는 역회전, 둘 다 LOW는 정지).
- 조립: 6핀 커넥터는 모터, 4핀 커넥터는 라이다에 연결한다. 라이다는 "라이다"라고 적힌 구멍에 지지대를 세워 동그란 쪽이 위로 가게 장착하고, 모터는 납작한 면이 나사와 만나도록 고정한다.
- 라이다 각도 기준은 뒤 0°, 왼쪽 90°, 앞 180°, 오른쪽 270°이다. 다음 과제인 장애물 회피 주행에서 앞쪽 거리를 판단할 때 이 기준을 사용한다.

### 4.2 4일차 (261122)

#### 4.2.1 다중 노드 파이프라인

하나의 기능을 입력, 연산, 출력 노드로 나누어 구성했다.

```
publisher ──/wasd──▶ processor ──/location──▶ subscriber
 (키 입력)            (위치 계산)               (결과 출력)
```

- **publisher**: WASD 키보드 입력을 받아 발행한다.
- **processor**: 연산을 담당한다. 받은 키에 따라 로봇의 위치를 계산하며, 시작 지점은 항상 (0, 0)으로 가정한다.
- **subscriber**: 계산 결과만 출력한다.

processor는 subscriber이면서 동시에 publisher이다. 역할을 노드 단위로 나누면 각 노드를 따로 교체하거나 재사용할 수 있다. 예를 들어 키보드 publisher를 실제 센서 노드로 바꿔도 processor와 subscriber는 그대로 쓸 수 있다.

메시지를 보낼 때는 객체를 생성한 뒤(`String()`, Python에서 괄호는 호출·실행을 의미하므로 클래스 이름 뒤에 괄호를 붙이면 객체가 만들어진다) 변수(`msg.data`)에 직접 접근해 값을 수정하고, 그 다음에 `msg` 자체를 발행한다.

#### 4.2.2 런치 파일

여러 노드를 실행하려면 터미널을 여러 개 열고 명령을 하나씩 입력해야 한다. 런치 파일은 이 번거로움을 줄이기 위해 만들어진 것이다.

- 여러 ROS2 노드를 한 번의 명령어로 실행하기 위한 설정 파일이다.
- 노드 실행에 필요한 설정을 통합적으로 관리한다.
- `src/<패키지>/launch/` 디렉토리에 저장한다.
- Python으로 작성한다.
- 이름이 `.py`로만 끝나면 일반 Python 파일과 구분하기 어려우므로 `~.launch.py` 또는 `~_launch.py` 형식을 쓴다. 이 수업에서는 `~.launch.py`를 사용한다.

런치 파일은 `generate_launch_description()` 함수에서 실행할 항목을 `LaunchDescription`으로 묶어 반환한다. `ros2 launch <패키지> <파일>`로 실행하려면 런치 파일이 `install/` 아래에 복사되어 있어야 하므로, `setup.py`의 `data_files`에 launch 폴더를 등록해야 한다.

```python
('share/' + package_name + '/launch', ['launch/my_launch.launch.py']),
```

#### 4.2.3 서비스

- 노드 간 요청과 응답을 기반으로 하는 통신 방식이다.
- 클라이언트-서버 구조로 동작한다.
  - 클라이언트: 작업을 요청하고 응답을 받는다.
  - 서버: 요청을 처리하고 결과를 반환한다.
- 일회성 요청이나 짧은 작업에 적합하다. 예: 센서 데이터 요청, 로봇 상태 조회, 특정 값 초기화
- 요청과 응답의 데이터 구조는 `.srv` 파일로 정의한다.

코드에서는 서버가 `create_service(타입, 서비스 이름, 콜백)`으로 서비스를 열고, 콜백에서 `request`를 읽어 `response`를 채운 뒤 반환한다. 클라이언트는 `create_client(타입, 서비스 이름)`으로 클라이언트를 만들고, `wait_for_service()`로 서버가 준비될 때까지 기다린 다음 `call_async()`로 요청을 보낸다. `spin_until_future_complete()`는 응답이 도착할 때까지 노드를 실행한다.

#### 4.2.4 액션과 서비스 비교

액션은 장시간 수행되는 작업을 위한 비동기 통신 방식이며, 서비스처럼 클라이언트-서버 구조로 동작한다. 서비스와 달리 작업 중간에 피드백을 보내고, 작업 취소를 지원한다.

| | 서비스 | 액션 |
| --- | --- | --- |
| 흐름 | 요청 → 응답 | 골(goal) → 피드백 → 결과 |
| 용도 | 특정 작업 요청 및 결과 확인 | 장시간 작업 실행 및 관리 |
| 피드백 | 제공하지 않음 | 지원 |
| 취소 | 별도 취소 기능 없음 | 취소 요청 지원 |
| 정의 파일 | `.srv` | `.action` |

#### 4.2.5 사용자 정의 인터페이스

인터페이스는 ROS2 노드 간 통신에 사용되는 데이터 구조이다. `std_msgs`, `geometry_msgs` 같은 기본 인터페이스로 표현하기 어려운 데이터는 직접 정의할 수 있다.

- 커스텀 메시지(`.msg`): 메시지의 필드 이름과 데이터 타입을 정의한다.
- 커스텀 서비스(`.srv`): `---`를 기준으로 위에는 요청, 아래에는 응답 필드를 정의한다.

```
# MoveRobot.srv
RobotPose current_pose
float64 distance
---
RobotPose result_pose
bool success
```

인터페이스 파일은 빌드할 때 Python과 C++에서 쓸 수 있는 코드로 변환되므로, 인터페이스 패키지는 Python 패키지가 아니라 CMake(`ament_cmake`) 패키지로 만든다.

- `CMakeLists.txt`: `rosidl_generate_interfaces()`에 변환할 `.msg`, `.srv` 파일을 등록한다.
- `package.xml`: 빌드 도구 `rosidl_default_generators`, 실행 의존성 `rosidl_default_runtime`, 그룹 `rosidl_interface_packages`를 지정한다.
- 인터페이스를 사용하는 패키지(`2_srvmsg`)의 `package.xml`에 `<depend>custom_interfaces</depend>`를 추가해야 colcon이 빌드 순서를 올바르게 정한다.

정의한 메시지는 다른 메시지나 서비스 안에서 다시 사용할 수 있다. `AddTwoInts.srv`는 `Num` 메시지를, `MoveRobot.srv`는 `RobotPose` 메시지를 필드로 사용한다. 3일차에 배운 메시지 계층화와 재활용을 직접 적용한 것이다.

#### 4.2.6 2차원 로봇 이동 계산

가상의 2차원 로봇은 좌표(x, y)와 방향(θ)을 가진다. 현재 방향 θ로 거리 d만큼 직진하면 새 위치는 다음과 같다.

```
x' = x + d · cos θ
y' = y + d · sin θ
θ' = θ
```

Python의 `math.cos()`, `math.sin()`은 라디안을 입력으로 받는다. 도(°) 단위 각도를 쓰려면 `math.radians()`로 변환해야 한다(30° ≈ 0.5236 rad, 90° ≈ 1.5708 rad).

---

## 5. 실습 내용 및 결과 분석

모든 실습은 의도한 대로 동작하는 것을 확인했다. 각 실습의 출력 예시는 코드의 `get_logger()` 출력 형식에 따라 정리했으며, `<timestamp>`는 실행 시각이다.

### 5.1 [3일차] ROS2 예제 노드와 CLI — `demo_nodes_cpp`

- **사용 이론**: 노드, 토픽, publisher/subscriber, ROS2 CLI
- **동작**: `talker`가 `/chatter` 토픽에 `Hello World: <번호>` 문자열을 1초마다 발행하고, `listener`가 이를 받아 출력한다.
- **결과 분석**
  - `ros2 node list`에서 `/talker`, `/listener` 노드를, `ros2 topic list`에서 `/chatter` 토픽을 확인했다.
  - `ros2 topic echo /chatter`로 listener 없이도 토픽 내용을 볼 수 있다. 내가 발행한 메시지가 제대로 나가는지 확인할 때 유용하다.
  - `rqt_graph`에서 `talker → /chatter → listener` 연결을 그림으로 확인할 수 있다.

### 5.2 [3일차] turtlesim 토픽 발행과 서비스 목록

- **사용 이론**: `Twist` 메시지 구조, `ros2 topic pub`, 서비스
- **동작**: `turtlesim_node`를 실행하고 `/turtle1/cmd_vel`에 `Twist`를 발행해 거북이를 움직인다. `ros2 service list`로 turtlesim이 제공하는 서비스를 확인한다.
- **결과 분석**
  - `linear.x`는 거북이가 바라보는 방향으로의 전진 속도, `angular.z`는 회전 속도이므로 두 값을 함께 주면 거북이가 원호를 그리며 이동한다.
  - `--once`는 한 번만 발행하므로 거북이가 잠깐 움직이고 멈춘다. `-r 10`처럼 주기적으로 발행하면 계속 움직인다.
  - 서비스 목록에는 `/spawn`, `/clear`, `/reset` 같은 서비스가 있으며, 서비스도 토픽처럼 각자의 타입을 가진다.

### 5.3 [3일차] talker / listener — `261121/1_pubsub`

- **사용 이론**: Colcon 워크스페이스, `setup.py`의 `entry_points`, `Node` 상속, 타이머, `spin`
- **동작**
  - `talker.py`: 1초 타이머마다 시작 후 경과 시간(초)을 담아 `/demo_topic`에 발행한다.
  - `listener.py`: `/demo_topic`을 구독하고 받은 `msg.data`를 출력한다.
- **출력 형식**

  ```
  [INFO] [<timestamp>] [talker]: Publishing: "Hello ROS2: 3"
  [INFO] [<timestamp>] [listener]: Received: "Hello ROS2: 3"
  ```

- **결과 분석**
  - talker와 listener가 같은 토픽 이름과 타입(`/demo_topic`, `String`)을 사용하므로 메시지가 전달된다.
  - `demo_nodes_cpp`의 talker/listener는 `/chatter`를 사용하므로, 이 패키지의 노드와는 서로 메시지를 주고받지 않는다.
  - `setup.py`에 등록한 명령 이름(`talker`, `listener`)으로 `ros2 run 1_pubsub talker`처럼 실행할 수 있다.

### 5.4 [4일차] WASD 위치 계산 — `publisher.py`, `processor.py`, `subscriber.py`

- **사용 이론**: 다중 노드 파이프라인, 한 노드에서 구독과 발행 동시 사용
- **동작**
  - `publisher.py`: `input()`으로 키를 입력받아 소문자로 바꾼 뒤 `/wasd`에 발행한다.
  - `processor.py`: `/wasd`를 구독해 `w`(y+1), `a`(x−1), `s`(y−1), `d`(x+1)로 위치를 갱신하고, 이동 전후 좌표를 `/location`에 발행한다. 다른 키는 경고만 출력하고 발행하지 않는다.
  - `subscriber.py`: `/location`을 구독해 결과를 출력한다.
- **출력 형식** (시작 위치에서 `w` 입력)

  ```
  [INFO] [<timestamp>] [subscriber]: New Location: "((0, 0)) => ((0, 1))"
  ```

- **결과 분석**
  - 키를 입력할 때마다 processor가 위치를 누적해서 계산하고, subscriber에 이동 전후 좌표가 출력되는 것을 확인했다.
  - processor는 위치를 `numpy` 배열로 저장하고, 이동 전 좌표를 `copy()`로 따로 저장한다. `copy()` 없이 대입하면 같은 배열을 가리키게 되어 이동 전 좌표도 함께 바뀐다.
  - `publisher.py`는 0.1초 타이머 콜백 안에서 `input()`을 호출한다. `input()`은 Enter를 누를 때까지 실행을 멈추므로, 실제 발행 주기는 타이머가 아니라 키 입력 속도로 정해진다.
  - `.lower()`로 대문자 입력도 처리하므로 Caps Lock이 켜져 있어도 동작한다.
  - `rqt_graph`로 보면 `/publisher → /wasd → /processor → /location → /subscriber`로 이어지는 구조이며, processor 하나가 구독과 발행을 동시에 하는 것을 확인할 수 있다.

### 5.5 [4일차] 런치 파일 — `talk_listen.launch.py`, `my_launch.launch.py`

- **사용 이론**: 런치 파일, `setup.py`의 `data_files`
- **동작**
  - 각 노드를 `ExecuteProcess`로 `gnome-terminal`을 열어 실행한다. 새 터미널에서는 `install/setup.bash`를 `source`한 뒤 `ros2 run`을 실행하고, `exec bash`로 노드가 끝나도 터미널을 유지한다.
  - `RegisterEventHandler`와 `OnProcessStart`로 앞 노드의 프로세스가 시작되면 다음 노드를 실행한다(publisher → processor → subscriber).
- **결과 분석**
  - 명령 한 번으로 노드마다 터미널이 하나씩 열리는 것을 확인했다.
  - publisher는 `input()`으로 키보드 입력을 받아야 하므로 자기만의 터미널이 필요하다. 그래서 노드 출력을 한 화면에 모으는 일반적인 방식 대신 노드마다 터미널을 여는 방식을 사용했다.
  - 런치 파일 경로를 `data_files`에 등록하지 않으면 빌드해도 `install/`에 복사되지 않아 `ros2 launch`가 파일을 찾지 못한다.
  - 토픽의 기본 Durability는 Volatile이므로, 구독자가 연결되기 전에 발행된 메시지는 전달되지 않는다. 런치 파일에서는 publisher가 먼저 실행되지만 키 입력을 기다리는 동안에는 아무것도 발행하지 않으므로, processor와 subscriber가 연결된 뒤에 첫 메시지가 나가 유실이 생기지 않는다.

### 5.6 [4일차] 두 수 더하기 서비스 — `add_ints_server.py`, `add_ints_client.py`

- **사용 이론**: 서비스(클라이언트-서버), 커스텀 메시지 `Num`, 커스텀 서비스 `AddTwoInts`
- **동작**
  - 서버: `/add_two_ints` 서비스를 열고, 요청의 `a.my_num + b.my_num`을 `sum.my_num`에 담아 응답한다.
  - 클라이언트: 명령행 인자 두 개를 정수로 바꿔 요청을 보내고, 응답의 합을 출력한다.
- **출력 형식** (`add_ints_client 3 5`)

  ```
  [INFO] [<timestamp>] [srv_server]: Incoming request
  a: 3, b: 5
  [INFO] [<timestamp>] [srv_server]: Sending back response: [8]
  [INFO] [<timestamp>] [srv_client]: Sum: 8
  ```

- **결과 분석**
  - `Num` 메시지 안에 값이 들어 있으므로 `request.a.my_num`처럼 최종 변수까지 타고 들어가야 값을 읽을 수 있다.
  - 서버가 실행되기 전에 클라이언트를 실행하면 `Waiting for service...`를 출력하며 기다리다가, 서버가 켜지면 바로 요청을 보낸다.
  - 클라이언트는 응답을 받으면 종료되고, 서버는 `spin`으로 계속 다음 요청을 기다린다. 토픽과 달리 요청이 있을 때만 통신이 일어난다.

### 5.7 [4일차] 2차원 로봇 이동 서비스 — `move_robot_server.py`, `move_robot_client.py`

- **사용 이론**: 서비스, 커스텀 메시지 `RobotPose`, 커스텀 서비스 `MoveRobot`, 2차원 이동 계산
- **동작**
  - 클라이언트: 명령행 인자 x, y, theta, distance를 요청에 담아 보낸다.
  - 서버: 4.2.6의 식으로 새 위치를 계산해 `result_pose`에 담고, `success = True`로 응답한다. 이번 실습에서는 모든 이동이 성공하는 것으로 가정했다.
- **출력 형식** (`move_robot_client 0 0 0.5236 1.2`, 30° 방향으로 1.2 이동)

  ```
  [INFO] [<timestamp>] [srv_client]: Success: True
  [INFO] [<timestamp>] [srv_client]: Result X: 1.04
  [INFO] [<timestamp>] [srv_client]: Result Y: 0.60
  [INFO] [<timestamp>] [srv_client]: Result Theta: 0.52
  ```

- **결과 분석**
  - 1.2 × cos 30° ≈ 1.04, 1.2 × sin 30° = 0.60으로 계산 결과와 일치한다.
  - 응답이 성공 여부와 x, y, theta로 구성되어 있어, 클라이언트는 이동 후 로봇의 현재 위치를 바로 알 수 있다.
  - 직진만 하므로 theta는 요청 값이 그대로 응답된다.

---

## 6. 한계점 및 개선 방향

아래 항목은 모두 실습 결과에는 영향을 주지 않으며, 입력 방식이나 출력 모양에 관한 개선점이다.

1. **move_robot 서버의 각도 단위**: 서버는 theta를 라디안으로 계산하므로, `0 0 90 1`처럼 도 단위로 입력하면 결과가 틀린다. 라디안(90° → 1.5708)으로 입력하면 정상적으로 동작한다. 도 단위로 입력받으려면 서버에서 `theta = math.radians(request.current_pose.theta)`로 변환해 계산하면 된다. 또한 `math.cos(theta)`, `math.sin(theta)`만 단독으로 있는 두 줄은 결과를 사용하지 않으므로 지워도 된다.
2. **processor 출력 괄호**: `f"({prev[0], prev[1]})"`처럼 중괄호 안에 쉼표를 넣으면 튜플로 출력되어 `((0, 0)) => ((0, 1))`처럼 괄호가 두 겹으로 나온다. `f"({prev[0]}, {prev[1]})"`처럼 값을 따로 쓰면 `(0, 0) => (0, 1)`로 출력된다.
3. **publisher 로그 따옴표**: 로그 문자열 `f'Publishing: "{msg.data}'`에 닫는 따옴표가 빠져 있어 출력에서 따옴표 하나가 빠진다. `f'Publishing: "{msg.data}"'`로 고치면 된다.
