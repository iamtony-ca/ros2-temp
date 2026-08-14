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
