올려주신 토픽 리스트를 기준으로, ros2_calib 툴에서 캘리브레이션을 진행하기 위해 녹화해야 할 가장 최적의 토픽 2개는 다음과 같습니다.
 * 이미지 토픽 (택 1): * /zed_front/left/color/rect/image/compressed (추천)
   * /zed_front/left/color/rect/image
 * 카메라 정보 토픽: * /zed_front/left/color/rect/camera_info
선택 이유 및 권장 사항
 * Left (왼쪽 렌즈): 스테레오 카메라 중 한쪽 렌즈만 LiDAR와 매칭하면 되므로, 기준 렌즈인 left를 사용합니다.
 * Color (컬러): 흑백(Gray)보다 컬러 화면이 작업자가 UI에서 주변 환경의 특징점(상자 모서리, 포스터 가장자리 등)을 육안으로 구별하고 클릭하기 훨씬 수월합니다.
 * Rect (왜곡 보정): 캘리브레이션 알고리즘(PnP)은 렌즈 왜곡이 펴진 이미지를 가정하고 계산하므로 반드시 rect(rectified) 데이터를 써야 합니다.
 * Compressed (압축): 4개의 카메라 영상을 동시에 원본(raw) 토픽으로 녹화하면 데이터 병목(I/O Bottle-neck)이 발생하거나 rosbag 용량이 수십 GB 단위로 순식간에 늘어납니다. ros2_calib은 CompressedImage 타입을 완벽히 지원하므로 압축된 이미지 토픽을 녹화하시는 것을 강력히 추천합니다. (참고로 /zed_front/left/color/rect/image/camera_info는 중복 토픽이므로 위에서 안내한 기본 camera_info를 사용하시면 됩니다.)
최종 녹화 명령어 적용 예시
앞서 말씀드린 LaserScan -> PointCloud2 변환 토픽(cloud_front, cloud_rear)과 결합하면, rosbag 녹화 명령어는 다음과 같이 구성하시면 됩니다.
ros2 bag record -s mcap \
/tf /tf_static \
/cloud_front /cloud_rear \
/zed_front/left/color/rect/image/compressed \
/zed_front/left/color/rect/camera_info \
/zed_back/left/color/rect/image/compressed \
/zed_back/left/color/rect/camera_info \
/zed_left/left/color/rect/image/compressed \
/zed_left/left/color/rect/camera_info \
/zed_right/left/color/rect/image/compressed \
/zed_right/left/color/rect/camera_info

