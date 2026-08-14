좋습니다, 이제 명확하네요. `nvme0n1` 7.3T가 내장 부팅 디스크(8TB), `sda` 931.5G가 외장 케이스의 1TB SSD입니다. 용량도 인터페이스도 완전히 달라서 헷갈릴 여지가 없습니다.

`/dev/sda`로 진행하시면 됩니다.

```bash
sudo wipefs -a /dev/sda
sudo parted /dev/sda --script mklabel gpt mkpart primary ext4 0% 100%
sudo mkfs.ext4 -L XFER /dev/sda1

sudo mkdir -p /media/xfer
sudo mount /dev/sda1 /media/xfer
sudo chown sim1:sim1 /media/xfer

lsblk -f /dev/sda
id -u
```

마지막 두 줄로 결과 확인과 UID 확인을 같이 하시면 됩니다. `sda1 ext4 XFER <UUID> /media/xfer`가 뜨면 완료입니다.

참고로 `sda`로 잡혔다는 건 USB 브리지 모드(10G 이하)라는 뜻입니다. 40G 케이블을 쓰셨는데도 이렇게 나온다면 포트가 USB4가 아닐 가능성이 큽니다. 나중에 실제 복사 속도가 1GB/s 근처에서 막히면 `lsusb -t`로 링크 속도를 확인해보세요 — 대용량 전송이 목적이니 40G가 나오면 시간이 꽤 절약됩니다.

##
##
##
네, 그 순서 맞습니다. 정확히는 이렇게 됩니다:

**지금 PC2에서:**
```bash
sudo umount /media/xfer
udisksctl power-off -b /dev/sda
```
그 다음 외장 케이블 뽑고 → 전원 끄고 → SSD 교체(ssd2 빼고 ssd1 장착) → ssd1으로 부팅.

**PC1(ssd1) 부팅 후:**
```bash
id -u                          # sim1과 같은지 비교
lsblk -o NAME,SIZE,MODEL,TRAN  # 외장이 sda인지 확인 (내장 구성이 다르니 이름 바뀔 수 있음)

sudo mkdir -p /media/xfer
sudo mount /dev/sda1 /media/xfer
```

여기서 **장치명 재확인이 중요합니다.** PC1의 내장 디스크가 SATA면 그게 `sda`를 차지하고 외장이 `sdb`가 될 수 있어요. `MODEL`과 `931.5G`로 대조하세요.

**복사 전 용량 파악:**
```bash
du -sh ~/docker/isaac-sim-601/* ~/.cache/ov/hub
df -h /media/xfer
```

100GB 중 어디가 캐시인지 보고 제외 여부를 정하시면 됩니다.

**복사:**
```bash
docker diff isaac-sim-601 | head -50     # 컨테이너 변경분 있는지
docker stop isaac-sim-601
docker commit isaac-sim-601 isaac-sim-cc:6.0.1-snapshot

docker save isaac-sim-cc:6.0.1-snapshot | zstd -T0 -3 -o /media/xfer/isaac-sim-cc-601.tar.zst

sudo rsync -aHAX --numeric-ids --info=progress2 \
  --exclude='cache/' --exclude='logs/' \
  ~/docker/isaac-sim-601/ /media/xfer/isaac-sim-601/

sudo rsync -aHAX --numeric-ids --info=progress2 \
  ~/.cache/ov/hub/ /media/xfer/ov-hub/
```

**마무리:**
```bash
sync && sudo umount /media/xfer
```

**한 가지 조언**: ssd1을 앞으로 안 쓰신다고 해도, PC2에서 정상 동작을 확인할 때까지는 ssd1을 지우거나 포맷하지 마세요. 뭔가 빠뜨렸을 때 다시 꺼내올 수 있는 유일한 원본입니다. Isaac Sim 컨테이너가 PC2에서 제대로 뜨고 GUI까지 확인된 뒤에 정리하시는 걸 권합니다.

그리고 외장 1TB에 100GB만 쓰는 거라면, 이 기회에 `~/` 아래 다른 작업 데이터(ROS 2 워크스페이스, 데이터셋, `.ssh`, `.gitconfig` 등)도 같이 담아두시면 왕복을 줄일 수 있습니다.



##
##
##
##
##
자동 마운트됐네요. 목적지는 `/media/amr/XFER` 입니다.

```bash
sudo rsync -aHAX --numeric-ids --info=progress2 \
  ~/docker/isaac-sim-601 ~/ros2_ws \
  /media/amr/XFER/
```

(옮길 폴더는 실제 필요한 것으로 바꾸시고, 소스 끝에 슬래시 붙이지 않는 것만 지키세요.)

**시작 전 두 가지 확인:**

```bash
df -h /media/amr/XFER          # 여유 공간
du -sh ~/* ~/.??* 2>/dev/null | sort -h | tail -20   # 뭐가 큰지
```

두 번째 명령으로 홈 디렉터리에서 용량 큰 항목들이 정렬돼 나옵니다. 목록 보시고 무엇을 가져갈지 정하시면 됩니다. `.cache`처럼 안 옮겨도 되는 것들이 꽤 클 거예요.

**복사 끝나면:**

```bash
sync
udisksctl unmount -b /dev/sda1
udisksctl power-off -b /dev/sda
```

자동 마운트된 거라 `umount` 대신 `udisksctl unmount`가 깔끔합니다 (파일 관리자의 ⏏ 버튼과 동일).

한 가지 — `~/docker/isaac-sim-601` 안의 `cache`와 `logs`는 재생성되는 캐시라 제외하면 시간이 크게 줄어듭니다. 다만 폴더를 통째로 지정하면서 일부만 빼려면 `--exclude`를 쓰셔야 하는데, 여러 소스를 나열할 때는 패턴이 모든 소스에 적용되니 주의하세요. 헷갈리면 isaac-sim-601만 따로 한 줄로 돌리시는 게 안전합니다.


##
##
##
##
네, 순서 정확합니다. 정리하면:

**ssd1에서:** commit → save → 볼륨 rsync → 언마운트  
**ssd2에서:** load → 볼륨 rsync (외장 → 홈) → 권한/ACL 확인 → run

## ssd2에서의 주의점 하나

`~/docker/isaac-sim-601`이 **이미 존재합니다.** ssd2에서 이미지 빌드하실 때 그 디렉터리들을 만드셨을 테니까요. rsync가 덮어쓰기 전에 확인하세요:

```bash
ls -la ~/docker/isaac-sim-601/
du -sh ~/docker/isaac-sim-601/
```

비어 있으면 그냥 덮어쓰시면 되고, 뭔가 있으면 백업하거나 이름을 바꿔두세요. 어차피 컨테이너를 만든 적이 없다니 비어 있을 가능성이 높습니다.

```bash
sudo rsync -aHAX --numeric-ids --info=progress2 \
  /media/sim1/XFER/isaac-sim-601/ ~/docker/isaac-sim-601/
```

(소스 끝 슬래시 있음 — 내용물을 기존 디렉터리 안으로)

## run 직전 체크리스트

```bash
id -u                                        # 1000인지
ls -ln ~/docker/isaac-sim-601/               # 소유권 1234인지
getfacl ~/docker/isaac-sim-601/volume | head # ACL 살아있는지
ls ~/.cache/ov/hub                           # hub 캐시 있는지 ★
docker images | grep isaac                   # load된 이미지 확인
nvidia-smi                                   # 드라이버
```

★ **`~/.cache/ov/hub`는 `~/docker` 밖에 있습니다.** 별도로 옮기셔야 해요. ssd1에서 아직 안 하셨으면 지금 챙기세요:

```bash
# ssd1
sudo rsync -aHAX --numeric-ids ~/.cache/ov/hub /media/amr/XFER/ov-hub-backup/

# ssd2
mkdir -p ~/.cache/ov
sudo rsync -aHAX --numeric-ids /media/sim1/XFER/ov-hub-backup/hub/ ~/.cache/ov/hub/
```

## 그리고 run 전에

```bash
xhost +local:
```

이건 재부팅할 때마다 필요합니다. 빠뜨리면 GUI가 안 뜨고 X11 연결 오류가 납니다.

## 첫 실행 후 확인

```bash
docker exec isaac-sim-601 ls /opt/ros/          # ROS 2 살아있나
docker exec isaac-sim-601 id                    # 1234인지
docker exec isaac-sim-601 touch /isaac-sim/volume/test.txt
ls -l ~/docker/isaac-sim-601/volume/test.txt    # 호스트에서 보이고 편집 가능한지
```

마지막 테스트가 ACL이 제대로 동작하는지 확인하는 가장 빠른 방법입니다. 호스트에서 그 파일을 지울 수 있으면 양방향 권한이 정상입니다.

**그리고 반복해서 말씀드리지만** — 이 모든 게 정상 동작하는 걸 확인할 때까지 ssd1은 그대로 두세요. GUI로 Isaac Sim이 실제로 뜨는 것까지 보신 뒤에 정리하시면 됩니다.
