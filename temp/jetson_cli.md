Jetson AGX Orin에 JetPack 6.2.1 설치를 완료하신 것을 축하드립니다! JetPack 6.x 시리즈는 최신 커널과 Ubuntu 22.04 기반으로 많은 변화가 있었는데요, 시스템이 정상적으로 설치되었는지 확인하고 NVMe 장치를 점검하는 핵심 명령어들을 정리해 드립니다.
1. JetPack 및 OS 설치 확인 명령어
설치된 JetPack 버전과 L4T(Linux for Tegra) 릴리스 정보를 확인하는 가장 확실한 방법들입니다.
① JetPack 버전 확인
JetPack 6.x부터는 패키지 관리자를 통해 버전을 확인하는 것이 가장 정확합니다.
sudo apt-cache show nvidia-jetpack

 * 확인 사항: Version: 6.2.1-...과 같은 출력이 나오는지 확인하세요.
② L4T(Jetson Linux) 버전 확인
JetPack 6.2.1은 보통 L4T 36.4.4 버전을 포함합니다.
cat /etc/nv_tegra_release

 * 확인 사항: # R36 (release), REVISION: 4.4 문구가 있는지 확인하세요.
③ 시스템 상태 종합 확인 (jtop 권장)
명령어는 아니지만, Jetson 사용자라면 필수인 jtop 툴을 설치하면 CPU/GPU 점유율, 온도, JetPack 버전, CUDA/TensorRT 버전을 한눈에 볼 수 있습니다.
# 설치 (pip가 없다면 sudo apt install python3-pip 먼저 실행)
sudo pip3 install -U jetson-stats

# 실행
jtop

 * 실행 후 7번 메뉴(INFO)를 누르면 모든 소프트웨어 스택 버전을 확인할 수 있습니다.
2. NVMe SSD 장착 및 인식 확인
NVMe를 추가 장착했다면 하드웨어적으로 인식되었는지, 그리고 마운트가 되었는지 확인해야 합니다.
① 하드웨어 인식 확인 (PCIe 연결)
NVMe는 PCIe 슬롯을 사용하므로 장치가 리스트에 뜨는지 확인합니다.
lspci | grep -i nvme

 * 문구가 한 줄이라도 출력된다면 하드웨어적으로는 정상 연결된 상태입니다.
② 디스크 장치 및 파티션 확인
운영체제가 해당 디스크를 장치 파일로 인식하고 있는지 확인합니다.
lsblk

 * nvme0n1이라는 이름의 장치가 보여야 합니다. 만약 OS를 NVMe에 설치했다면 MOUNTPOINTS가 /로 표시될 것이고, 추가 저장장치로만 쓴다면 용량 정보만 표시될 것입니다.
③ 디스크 상세 정보 및 용량 확인
sudo fdisk -l | grep /dev/nvme
df -h | grep /dev/nvme

 * df -h 명령어로 용량이 제대로 표시되지 않는다면, 아직 **포맷(Format)**이나 **마운트(Mount)**가 되지 않은 상태일 수 있습니다.
💡 팁: 만약 NVMe를 추가 저장소로 쓰고 싶다면?
lsblk에는 뜨는데 df -h에는 안 뜬다면 아래 순서로 진행하세요. (데이터가 삭제되니 주의하세요!)
 * 포맷: sudo mkfs.ext4 /dev/nvme0n1 (파티션이 있다면 nvme0n1p1)
 * 마운트 폴더 생성: mkdir ~/data
 * 마운트: sudo mount /dev/nvme0n1 ~/data
혹시 NVMe로 부팅을 시도하시거나, 특정 라이브러리(CUDA, TensorRT)의 작동 여부도 확인하고 싶으신가요?
