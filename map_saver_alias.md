네, 좋은 질문입니다. `map_saver_cli`의 동작 방식은 launch 파일과는 조금 다릅니다.

결론부터 말씀드리면, **launch 파일처럼 패키지 폴더를 자동으로 찾아주는 기능은 없습니다.** 하지만 셸(shell)의 기능을 활용하면 비슷하게 구현할 수 있습니다.

-----

### 왜 다르게 동작할까요? 🤔

  * **Launch 파일:** ROS 2의 기능을 활용하는 **Python 스크립트**입니다. 따라서 `get_package_share_directory` 같은 함수를 사용해 특정 패키지의 경로를 동적으로 가져와 조합하는 로직을 넣을 수 있습니다.
  * **`map_saver_cli`:** 독립적으로 실행되는 **C++ 실행 파일**입니다. 이 프로그램은 ROS 2 패키지 시스템을 알지 못하며, 파일 경로를 운영체제(Linux)의 파일 시스템 규칙에 따라 해석합니다. 즉, 경로가 `/`로 시작하지 않으면 \*\*명령어를 실행한 현재 위치(Current Working Directory)\*\*를 기준으로 파일을 저장하려고 합니다.

-----

### 해결 방법: 셸 스크립팅 활용하기 💡

`map_saver_cli`에 절대 경로를 매번 입력하는 것이 번거롭다면, `ros2 pkg prefix` 또는 `ros2 pkg share` 명령어와 셸 스크립팅을 조합하여 원하는 동작을 만들 수 있습니다.

#### 방법 1: `cd`를 이용해 저장할 위치로 미리 이동하기

가장 간단한 방법입니다. 맵을 저장하고 싶은 폴더로 먼저 이동한 뒤, `map_saver_cli`를 실행하면 됩니다.

```bash
# 내 패키지의 maps 폴더로 이동
cd $(ros2 pkg share my_robot_bringup)/maps

# 이제 맵 이름만 지정하여 저장
ros2 run nav2_map_server map_saver_cli -f my_new_map
```

위와 같이 실행하면 `my_robot_bringup/maps/` 폴더 안에 `my_new_map.yaml`과 `my_new_map.pgm` 파일이 저장됩니다.

#### 방법 2: 명령어 조합으로 절대 경로 동적으로 생성하기 (추천)

`cd`를 사용하기 번거롭다면, 파일 경로를 동적으로 만들어서 인자로 넘겨주는 것이 더 편리합니다.

```bash
# 'my_robot_bringup' 패키지의 'maps' 폴더에 'my_awesome_map' 이라는 이름으로 저장
ros2 run nav2_map_server map_saver_cli -f $(ros2 pkg share my_robot_bringup)/maps/my_awesome_map
```

  * `$(ros2 pkg share my_robot_bringup)`: `my_robot_bringup` 패키지의 `share` 폴더까지의 절대 경로를 반환합니다.
  * 결과적으로 `map_saver_cli`에는 `/path/to/your/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/maps/my_awesome_map`과 같은 전체 경로가 전달됩니다.

이 방식을 사용하면 어느 위치에서 명령어를 실행하든 항상 원하는 패키지의 `maps` 폴더에 맵을 저장할 수 있습니다. `.bashrc`에 alias로 등록해두고 사용하면 훨씬 편리합니다.

```bash
# ~/.bashrc 파일에 아래 내용 추가
# alias save_map_in_pkg='ros2 run nav2_map_server map_saver_cli -f $(ros2 pkg share my_robot_bringup)/maps/$1'

# 터미널 재시작 후 아래처럼 사용 가능
# save_map_in_pkg my_final_map
```
