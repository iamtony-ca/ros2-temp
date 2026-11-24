`rec` 안의 파일은 보관용 원본이라 직접 열어 보는 용도가 아닙니다. 원본에서 보고 싶은 시간 구간을 `nav_data_export`로 새 mcap 파일로 만들고, **그 새 파일을** 엽니다. 수동 표시 구간에 있는 `pinned/…/export.mcap`은 이미 이렇게 만든 파일이라 바로 열어도 됩니다.

| 파일 | Lichtblick으로 | 용도 |
|---|---|---|
| `~/ros2_logs/nav_data/rec/…/*.mcap` | ✗ costmap과 스캔이 압축 형태 | 7일 보관용 원본 |
| `nav_data_export`로 만든 파일 (예: `~/nav_export/test.mcap`) | ✓ | 분석용 |
| `~/ros2_logs/nav_data/pinned/…/export.mcap` | ✓ | 수동 표시 구간, 이미 내보낸 파일 |

**만드는 방법** (로봇 주행 PC 터미널, ROS 환경이 source된 상태)
```bash
mkdir -p ~/nav_export
# 특정 시각 앞뒤 1분
ros2 run nav_data_tools nav_data_export --root ~/ros2_logs/nav_data \
    --at "2026-11-24 15:02:10" --before 60 --after 60 -o ~/nav_export/test.mcap
# 또는 구간을 직접 지정
ros2 run nav_data_tools nav_data_export --root ~/ros2_logs/nav_data \
    --from "2026-11-24 14:55:00" --to "2026-11-24 15:10:00" -o ~/nav_export/test.mcap
```
- 시각은 꼭 지정해야 합니다. 파일만 주면 거부합니다. 처음 해 보실 때는 기록이 확실히 있는 최근 시각으로 1~2분 구간을 잡으시면 됩니다.
- 내보내는 과정에서 압축된 costmap과 스캔을 표준 메시지로 되돌립니다. 그래서 만든 파일에는 `/local_costmap/costmap`과 `/global_costmap/costmap`이 생깁니다.
- 그 다음 `lichtblick --defaultLayout=nav_data_analysis ~/nav_export/test.mcap`으로 엽니다. 노트북에서 보려면 이 파일만 옮기면 됩니다(2분이면 보통 몇 MB).

기록된 시간 범위를 모르겠으면 `ls ~/ros2_logs/nav_data/rec/*/`로 보세요. 폴더 이름이 `<시각>_<부팅ID>` 형태라 기록이 시작된 시각을 알 수 있습니다.
