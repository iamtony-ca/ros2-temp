옮기신 tar 파일이 어느 판인지 확인했습니다. 지금 현장 가이드를 정리해 드립니다.

**옮기신 `zed_e5_kit.tar`는 워치독 개선 전 판입니다(11-23 16:11 생성).** `Downloads/transfer/transfer_261123`와 `Downloads/transfer`에 있는 tar는 md5 `d1f0531f…`로 똑같습니다. 이 판은 E5를 적용하면서 워치독을 예전처럼 3개 토픽 감시로 둡니다. 워치독 개선은 말씀하신 대로 E5가 끝난 뒤 손으로 넣으시면 됩니다.

**대상:** 아직 예전 설정(SDK 5.1.1, 카메라 3대)인 호기만입니다. 5호기와 6호기에는 쓰지 마세요. 6호기에는 이미 `~/zed_e5_kit`가 있어서 덮어쓰면 안 되고, 5호기는 시험 표시가 남아 있습니다.

## 0. 시작 전
- 관제에서 그 호기를 운행 제외하고, 로봇은 정차한 상태여야 합니다.
- Orin에 모니터와 키보드를 연결하고 orin 계정으로 로그인합니다. 자동 실행 창 4개(zenoh, 워치독, laserscan, jetson_monitor)가 모두 뜰 때까지 기다립니다.
- 전체 45~90분 걸리고, 그중 카메라가 꺼지는 시간은 보통 10~25분입니다.
- 명령 앞에 `sudo`를 붙이지 않습니다. 각 단계가 끝나면 화면에 나오는 `NEXT:` 명령만 실행합니다.

## 1. 압축 풀기 (Ctrl+Alt+T로 터미널을 엽니다)
```
cd ~ && tar xf <tar 경로>/zed_e5_kit.tar      # ~/zed_e5_kit 이 생김
bash ~/zed_e5_kit/orin/onsite.sh check-files   # "all files OK" 확인
```

## 2. 업그레이드 (명령은 전부 `bash ~/zed_e5_kit/orin/onsite.sh <단계>` 형식)

| # | 단계 | OK 기준 / 할 일 |
|---|---|---|
| 1 | `preflight` | `RESULT: 0 FAIL`이 나와야 합니다. 시리얼 4개, DOMAIN, 호기 번호(= Orin DOMAIN − 50)를 적어 둡니다. 호기 번호가 작업 대상과 다르면 멈춥니다. |
| 2 | `backup` | `DONE: backup in /home/orin/zed_backup_…`이 나오면 그 경로를 적어 둡니다. |
| 3 | `upgrade` | `OOS`를 입력합니다. 카메라가 꺼집니다. `upgrade complete`가 나오면 바로 다음 단계로 갑니다. |
| 4 | `reboot` | 재부팅 뒤 로그인하고, 자동 실행 창 4개가 뜰 때까지 기다립니다. |
| 5 | `post` | `RESULT: PASS`가 나와야 합니다(토픽 5개 15 Hz). |
| 5a | `view-left` | 로봇 왼쪽 카메라를 손으로 가렸을 때 영상이 어두워져야 합니다. 확인 뒤 창을 꼭 닫습니다. |
| 6 | `verify 30` | `RESULT: PASS`가 나오면 끝입니다. 이 다음에 운행 복귀할 수 있습니다. |

- **preflight WARN 판단 기준:** 현장작업순서 카드의 표를 따릅니다. 캘리브레이션 파일 없음, left 카메라 미검출, DOMAIN 불일치가 나오면 멈춥니다.
- **실패하면:** 롤백 카드 순서(`rollback-dryrun` → `rollback` → `reboot` → `rollback-post`)를 따릅니다.
- **터미널을 닫았을 때:** `onsite.sh status`를 실행하고 나오는 `NEXT:`를 따릅니다.
- 자세한 내용은 압축을 푼 키트 안의 `~/zed_e5_kit/현장작업순서_ko.md`와 `README_ko.md`에 있습니다.

## 3. 워치독 개선을 손으로 적용 (verify PASS 뒤, 운행 제외 상태에서)
1. **노드 원본 확인:** `md5sum ~/zed_ws/src/zed_watchdog/zed_watchdog/zed_watchdog_node.py`가 `bcb05383fcb88e14aa77d4b149c135ed`인지 봅니다. 다르면 그 호기는 노드가 이미 다르니 멈추고 확인합니다.
2. **노드 교체:** 6호기의 git changes와 같은 파일로 바꿉니다. 바꾼 뒤 md5는 `646c2563977b42272d331229a4bd6a88`이어야 합니다. 노트북의 `~/orin_zed_rollback/fleet_e5/bundle/watchdog/watchdog_4cam.patch`를 가져갔다면 `cd ~/zed_ws/src && git apply <패치>`로 해도 됩니다.
3. **yaml 수정:** `zed_watchdog/config/zed_watchdog.yaml`은 git changes에 나오지 않으니 손으로 고칩니다.
   - right 토픽을 `/zed_multi/zed_right/point_cloud/cloud_registered`로 바꿉니다.
   - `/zed_multi/zed_left/point_cloud/cloud_registered`를 추가합니다.
   - `max_attempts: 10`으로 바꾸고, `attempt_reset_sec: 600.0`과 `fatal_retry_sec: 300.0`을 추가합니다.
4. **워치독 재시작:** `bash ~/zed_e5_kit/orin/restart_watchdog.sh`를 실행하고 `OOS`를 입력합니다. 카메라가 약 1분 꺼지고, 빌드는 필요 없습니다.
5. **확인:** `grep "WD4" ~/ros2_logs/python3_*.log | tail -1`에 `[WD4] watching 4 topics`가 나와야 합니다. point cloud 4개가 15 Hz인지도 봅니다.

**주의할 점 두 가지**
- **verify 보고서:** 이 판의 verify 보고서는 버그 때문에 워치독 이벤트 수가 항상 0으로 나옵니다. 끊김(stall) 수와 토픽 Hz로 판단하세요.
- **롤백:** 워치독 개선을 넣은 뒤 롤백하면 yaml은 원래대로 돌아가지만 노드 `.py`는 바뀐 채로 남을 수 있습니다. 바뀐 노드는 3개 설정에서도 그대로 동작하니 운행에는 문제없습니다.

원하시면 이 가이드를 호기마다 들고 다닐 한 장짜리 페이지로 만들어 드리겠습니다.
