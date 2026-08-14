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
