Seri, 결론부터 말하면 **MSI MAG Z890 TOMAHAWK WIFI의 유선 LAN(5G)**은 보통 **Killer E5000B 계열(실제로는 Realtek RTL8126 5GbE 쪽과 엮이는 경우가 많음)**이라서, Ubuntu 24.04.3 기본 커널/기본 드라이버만으로는 장치가 “아예 안 보이거나” 드라이버 매칭이 안 되는 케이스가 자주 나옵니다. 이 보드는 “5G LAN / Killer E5000B-CG”로도 표기됩니다. 

그리고 당신이 준 Linux Mint 포럼 링크는 JavaScript 없이는 본문 열람이 막혀 있어서(텍스트로 내용을 “정독”해서 인용하는 방식은) 제가 그대로는 확인이 불가능했습니다. 
대신, 동일 증상(와이파이는 되는데 이더넷이 아예 없음) + Killer E5000/RTL8126(5GbE) 드라이버 부재 패턴에 대해, 재현 가능한 해결책을 “Ubuntu 24.04 기준”으로 정리해줄게요.


---

1) 먼저 10초 진단 (내 NIC가 뭔지 확정)

와이파이로 연결된 상태에서 아래만 실행해봐.

lspci -nnk | grep -A3 -i 'ethernet\|network'
sudo lshw -C network
dmesg | egrep -i 'r8169|r8126|ether|kiler|realtek|e5000'

출력에 “Killer E5000” 또는 Realtek 계열(10ec:xxxx) 이 보이는데 “Kernel driver in use:”가 비어있거나 / r8169만 잡히거나 / 아예 장치가 안 뜨면 → 아래 “r8126(Realtek 5GbE) 드라이버” 쪽이 정답일 확률이 높습니다.

“Killer E5000”은 리눅스에서 지원이 늦어졌다는 얘기가 공식/커뮤니티에 실제로 나옵니다. 



---

2) 가장 현실적인 해결: Realtek r8126(5GbE) 드라이버를 DKMS로 설치

이게 현재 Ubuntu 24.04에서 제일 성공률 높은 루트입니다. (커널이 올라가도 자동으로 재빌드되게 DKMS 사용)

(A) 준비물 설치

sudo apt update
sudo apt install -y build-essential dkms linux-headers-$(uname -r)

(B) Secure Boot 확인 (중요)

DKMS로 설치한 커널 모듈은 Secure Boot가 켜져 있으면 로드가 막힐 수 있어요.

mokutil --sb-state

SecureBoot enabled면: BIOS에서 Secure Boot를 끄는 게 제일 빠름(또는 MOK 등록/서명 절차 필요)


(C) Realtek 공식 r8126 드라이버 받기

Realtek의 5GbE(8126) Linux 드라이버는 여기(공식 다운로드 목록)에서 받습니다. 

https://www.realtek.com/Download/List?cate_id=584

브라우저로 받아서 ~/Downloads에 있다고 가정하고 진행할게요.

(D) 설치 (Realtek 패키지는 보통 autorun.sh 제공)

cd ~/Downloads
tar -xf r8126-*.tar.*   # 확장자가 tar.bz2 / tar.gz 등일 수 있음
cd r8126-*/

sudo ./autorun.sh

설치 후 모듈 로드:

sudo modprobe r8126

(E) (충돌 방지) r8169가 잡고 있으면 블랙리스트

기본 드라이버 r8169가 억지로 물고 있으면 r8126이 안 붙는 경우가 있습니다(Realtek 계열에서 흔함).

echo "blacklist r8169" | sudo tee /etc/modprobe.d/blacklist-r8169.conf
sudo update-initramfs -u

sudo modprobe -r r8169 2>/dev/null || true
sudo modprobe r8126
sudo systemctl restart NetworkManager

(F) 재부팅 후 확인

ip link
sudo ethtool -i <유선인터페이스명>   # 예: enp5s0

driver: r8126가 나오면 성공



---

3) 대안: 커널을 “지원되는 버전”으로 올리기 (단, 안정성/호환성 고려)

Killer E5000 지원이 리눅스 커널 6.15 사이클에서 들어간다는 보도가 있습니다. 
즉 “아무것도 설치하기 싫다, 커널만으로 해결하고 싶다”면 커널을 올리는 방법이 있지만:

Ubuntu 24.04 LTS에서 메인라인 커널(실험적)로 가는 건 리스크가 있어요.

그래서 실무적으로는 위의 r8126 DKMS 설치가 더 안전한 편입니다.



---

4) 그래도 안 되면 체크할 것 (짧게)

1. BIOS에서 Onboard LAN Enabled인지 (LAN 관련 옵션 OFF면 OS에서 영영 안 보임)


2. 케이블/스위치 포트 교체 (링크 LED 점등 여부)


3. sudo journalctl -u NetworkManager --no-pager -n 200 에서 유선 관련 에러 확인


4. lspci에는 뜨는데 ip link에 인터페이스가 아예 없다 → 드라이버 미매칭/secure boot/모듈 로드 실패 쪽이 거의 확정




---

원하면, 위 1) 진단 커맨드 출력(lspci -nnk …, lshw -C network, mokutil --sb-state)만 그대로 붙여줘. 그럼 네 보드에서 실제로 잡히는 PCI ID 기준으로 “r8126이 맞는지 / 다른 드라이버(예: igc)가 맞는지”를 딱 잘라서 확정해줄게.

##################  
###################
 #################  
좋아, 네가 준 포럼 전문을 보면 핵심은 딱 2줄이야:

보드의 유선 NIC가 lspci에 **Realtek 10ec:5000**로 잡히는데, 커널 기본 드라이버가 매칭이 안 됨(driver: N/A) → Mint/Ubuntu 계열에서 동일 증상 

해결은 Realtek 5GbE(r8126) 드라이버를 DKMS로 설치하는 쪽이 정답(포럼에서도 결국 그걸로 해결)이고, Secure Boot는 꺼야 함 


Ubuntu 24.04.3는 HWE 커널이 Linux 6.14로 올라간 릴리스라(새 하드웨어 지원 목적) 커널 업그레이드만으로 해결되길 기대하기 쉬운데, 이 케이스는 “커널만으로는 아직 부족 + 별도 r8126 드라이버가 필요” 쪽으로 보는 게 가장 안정적이야. 

아래가 Ubuntu 24.04.3에서 최적(성공률/유지관리/재부팅 최소) 루트야.


---

✅ 최적 해법: awesometic realtek-r8126-dkms(최신) 설치 + Secure Boot OFF

0) 내 NIC가 정말 10ec:5000인지 확인

lspci -nnk | grep -A3 -i ethernet
uname -r
mokutil --sb-state

Ethernet controller ... [10ec:5000]면 포럼과 동일 케이스

SecureBoot enabled면 BIOS에서 Secure Boot 끄는 게 제일 빠름 (DKMS 모듈 로딩이 막힐 수 있음)


1) 빌드/헤더 준비

sudo apt update
sudo apt install -y build-essential dkms linux-headers-$(uname -r)

2) DKMS 드라이버(최신 릴리스) 설치

포럼에 나온 건 10.015.00-1인데, 지금은 10.016.00-1 최신 릴리스가 있음(Realtek r8126 드라이버 업데이트). 

cd ~/Downloads
wget -O realtek-r8126-dkms_10.016.00-1_amd64.deb \
  https://github.com/awesometic/realtek-r8126-dkms/releases/download/10.016.00-1/realtek-r8126-dkms_10.016.00-1_amd64.deb

sudo apt install ./realtek-r8126-dkms_10.016.00-1_amd64.deb

3) 모듈 로드 + 네트워크 재시작

sudo modprobe r8126
sudo systemctl restart NetworkManager
ip link

4) 드라이버가 제대로 붙었는지 확인

ip link에서 새로 생긴 유선 인터페이스(enp… 같은)가 보이면:

sudo ethtool -i <유선인터페이스명>

여기서 driver: r8126 나오면 끝.


---

⚠️ 설치는 됐는데 “여전히 유선 인터페이스가 안 생길 때” (2가지 처방)

A) 혹시 기본 드라이버(r8169)가 잡아먹는 경우(드묾) → 블랙리스트

lsmod | grep r8169

r8169가 올라와 있으면:

echo "blacklist r8169" | sudo tee /etc/modprobe.d/blacklist-r8169.conf
sudo update-initramfs -u
sudo reboot

B) DKMS 빌드 에러가 날 때

포럼에서처럼 **메인라인 커널(예: 6.14.0-061400-generic)**에선 DKMS 빌드가 깨지는 경우가 있어. 
Ubuntu 24.04.3는 **“우분투가 패키징한 6.14 HWE”**를 쓰는 게 제일 안정적이니, 만약 네 커널이 061400 같은 mainline이면 우선 그걸 피하는 게 좋아.

지금 커널 확인:

uname -r

6.14.0-xx-generic (Ubuntu 패키지) → 그대로 진행

6.14.0-061400-generic (mainline) → Ubuntu 정식 HWE 커널로 되돌리는 쪽이 성공률↑



그리고 DKMS 에러 로그는 여기:

cat /var/lib/dkms/realtek-r8126/*/build/make.log | tail -n 80


---

대안(추천도 낮음)

Realtek 공식 r8126 소스 드라이버 직접 설치(업데이트 때마다 재작업 가능)
Realtek이 r8126 Linux 드라이버를 배포하고 있고, 커널 상한도 계속 갱신됨. 

급하면 USB-Ethernet 동글(포럼에서도 즉시 해결책으로 사용) 



---

원하면, 아래 3개 출력만 붙여줘. 그러면 “너는 mainline 커널인지 / secure boot 상태 / 실제로 어떤 모듈이 붙는지”를 기준으로 딱 네 PC에 맞춰서 한 번에 마무리 커맨드만 골라줄게.

lspci -nnk | grep -A3 -i ethernet
uname -r
mokutil --sb-state
```7
