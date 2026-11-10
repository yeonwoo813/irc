# 카메라 영상과 설정

카메라 확인·조절·저장은 `vision_stack.launch.py`에서 합니다.
`robot_bringup.py`는 기존 로봇 실행 구성을 유지하며 저장된 카메라 설정을
시작할 때 읽어 적용합니다. 작업 디렉터리 `~/irc`에서 미리보기를 실행합니다.

```bash
source install/setup.bash
ros2 launch robot_bringup vision_stack.launch.py show_preview:=true
```

Webcam / RealSense RGB 창에서 영상과 검출 결과를 보며 노출, 게인,
화이트밸런스, 밝기, 대비, 채도, 선명도를 조절합니다. 카메라가 지원하는
범위만 사용하고, 지원하지 않는 항목은 비활성화됩니다.

- `자동 노출`, `자동 화이트밸런스` 체크박스로 자동 모드를 선택합니다.
- 노출·게인 슬라이더를 움직이면 자동 노출을 끄고 수동값을 적용합니다.
  화이트밸런스 슬라이더는 자동 화이트밸런스를 끕니다.
- 각 항목 오른쪽의 큰 숫자는 최신 현재값입니다. `프레임`은 RealSense RGB
  프레임 메타데이터, `장치 조회`는 카메라가 조회 시점에 보고한 값입니다.
  자동 노출 중에는 노출·게인, 자동 화이트밸런스 중에는 색온도도 갱신합니다.
  밝기·대비·채도·선명도는 별도의 자동 모드가 없는 수동 보정 항목이며,
  동일하게 현재 적용값을 표시합니다.
- 자동 모드에서 최신값이 있으면 슬라이더와 숫자 입력도 현재값을 따라가며
  `조절 시 수동`으로 표시합니다. 최신값이 없을 때 `수동 설정`으로 표시된
  입력란은 저장된 수동값입니다. 자동 모드를 해제하면
  최신 현재값을 우선 이어받습니다. 읽지 못한 항목은 표시된 수동값으로
  전환하고 하단에 안내합니다. 입력값을 직접 바꾸면 입력한 값이 우선합니다.
- 2.5초 이상 새 값을 받지 못하거나 장치가 값을 제공하지 않으면 `—`와
  `현재값 미수신`을 표시합니다. 저장된 수동값을 실시간 값으로 표시하지 않습니다.
- `설정 저장 · 다음 실행에 적용`을 누르면 현재 드라이버 설정을 확인한 뒤
  선택된 INI에 저장합니다. 조절만 하고 저장하지 않은 값은 다음 실행에
  복원되지 않습니다. 자동 모드 저장 시에는 해당 모드를 저장하고 기존
  수동 설정값을 보존합니다.
- RealSense RGB 노출은 µs, 웹캠 노출은 100 µs 단위입니다.
  RealSense Depth는 별도의 기존 깊이 영상 창에서 조절합니다.

기본 저장 파일은 `src/vision/config/settings.ini`이며 웹캠은
`[webcam_controls]`, RealSense RGB는 `[realsense_rgb]`, 깊이 설정은
`[realsense_depth]`에 저장합니다. 각 창의 저장은 다른 설정과 주석을 보존합니다.

`vision_stack`에서 설정을 저장하고 실행을 종료한 뒤 전체 로봇을 실행합니다.
`robot_bringup`은 조절 창 없이 저장된 설정을 읽어 적용합니다.

```bash
ros2 launch robot_bringup robot_bringup.py
```

다른 설정 파일은 두 launch 모두 `settings_ini:=/절대경로/설정.ini`로
동일한 경로를 지정해야 합니다. 설정창의 `저장 위치 보기`에 마우스를 올리면
실제 저장 경로를 확인할 수 있습니다.
자동 모드로 저장하면 로봇 실행에서도 자동 모드가 적용됩니다. 조절한 숫자를
고정해서 사용하려면 `vision_stack`에서 자동 모드를 해제한 뒤 저장합니다.

검출 모델을 실행하지 않고 카메라 원본만 보며 조절할 수도 있습니다.

```bash
ros2 launch robot_bringup vision_stack.launch.py show_preview:=true \
  start_yolo:=false start_realsense_yolo:=false start_ball:=false start_hurdle:=false
```

새 기본값은 RGB 자동 노출과 자동 화이트밸런스를 계속 유지합니다.
기존 launch의 `lock_realsense_rgb_after_warmup`과
`realsense_rgb_warmup_seconds`는 체크박스와 저장 설정으로 대체했습니다.
YOLO가 켜져 있으면 설정은 즉시 적용하고, 영상 창은 기존처럼 모델 준비 후
열립니다. 창을 닫아도 검출 프로세스는 계속 실행됩니다.

RGB 제어는 카메라 드라이버의 ROS 파라미터 서비스를 사용합니다.
웹캠 이름과 지원 범위는 실행 시 검색하며 구형·신형 V4L2 이름을 지원합니다.
범위 검증, 서비스 성공 응답, 파라미터 재조회로 적용을 확인합니다.
실제 장치에 대한 최종 확인은 영상과 드라이버 진단 로그로 확인해야 합니다.
현재값 조회는 설정 변경과 분리되어 있습니다. RealSense는
`/camera/color/metadata`를 우선 사용하고, 이 저장소 드라이버의 읽기 전용
`/camera/get_rgb_controls` 서비스로 센서 옵션을 약 0.5초마다 조회합니다.
D400의 `manual_white_balance`는 수동 프리셋이고
`auto_white_balance_temperature`는 자동 모드 플래그이므로 실제 색온도로
사용하지 않습니다. 장치 조회는 프레임에 정확히 동기화된 측정값은 아니며,
장치/펌웨어가 자동 모드의 현재값 조회를 지원하는 범위에서 표시합니다.
웹캠은 드라이버의 `video_device`를 읽고 별도 작업 스레드에서 읽기 전용
`VIDIOC_G_CTRL`로 조회합니다. 새 영상 스트림을 시작하지 않으며,
미리보기 없는 bringup에서는 현재값 조회를 실행하지 않습니다.

변경된 드라이버와 설정창을 함께 빌드합니다.

```bash
colcon build --packages-select realsense2_camera robot_bringup
source install/setup.bash
```

V4L2 드라이버의 이름 변환 규칙은
[공식 설명](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/tree/rolling)을 참고하세요.

테스트는 별도의 로컬 ROS 도메인에서 가상 드라이버와 화면 없는 Qt를 사용합니다.

```bash
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD/src/robot_bringup:$PYTHONPATH" \
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=176 QT_QPA_PLATFORM=offscreen \
  python3 -m pytest -q src/robot_bringup/test
```
