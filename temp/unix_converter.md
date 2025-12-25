네, `zoneinfo`를 활용하여 **Unix Timestamp ↔ Seoul Time(KST)** 양방향 변환이 가능한 깔끔하고 견고한 코드를 작성해 드립니다.

로보틱스 개발 환경(Ubuntu 24.04)을 고려하여, **Type Hinting**을 적용하고 **PEP 8** 스타일을 준수한 모듈 형태로 구성했습니다.

### Python Code: Time Converter Utility

```python
from datetime import datetime
from zoneinfo import ZoneInfo
import time

# 상수 정의 (포맷 변경 시 이곳만 수정하면 됩니다)
SEOUL_TZ = ZoneInfo("Asia/Seoul")
DEFAULT_FMT = '%Y-%m-%d %H:%M:%S'

class TimeConverter:
    """
    Unix Timestamp와 Seoul Time(Asia/Seoul) 간의 양방향 변환 유틸리티
    """

    @staticmethod
    def unix_to_seoul(unix_timestamp: float, fmt: str = DEFAULT_FMT) -> str:
        """
        Unix Timestamp(float) -> Seoul Time 문자열 변환
        """
        # 타임스탬프를 서울 타임존을 가진 aware datetime 객체로 변환
        dt_seoul = datetime.fromtimestamp(unix_timestamp, SEOUL_TZ)
        return dt_seoul.strftime(fmt)

    @staticmethod
    def seoul_to_unix(time_str: str, fmt: str = DEFAULT_FMT) -> float:
        """
        Seoul Time 문자열 -> Unix Timestamp(float) 변환
        주의: 입력 문자열은 서울 시간이라고 가정합니다.
        """
        # 1. 문자열 파싱 (Naive datetime 생성)
        dt_naive = datetime.strptime(time_str, fmt)
        
        # 2. Naive datetime에 서울 타임존 정보 주입 (Aware datetime 변환)
        # replace를 사용하여 시각 정보는 유지한 채 타임존만 할당
        dt_aware = dt_naive.replace(tzinfo=SEOUL_TZ)
        
        # 3. Unix Timestamp 반환
        return dt_aware.timestamp()

# --- Main 실행 예제 ---
if __name__ == "__main__":
    # 1. 현재 시간을 기준으로 테스트
    current_ts = time.time()
    
    print(f"--- [1] Unix ({current_ts:.2f}) -> Seoul Time ---")
    seoul_str = TimeConverter.unix_to_seoul(current_ts)
    print(f"Result: {seoul_str} (KST)")

    print(f"\n--- [2] Seoul Time ('{seoul_str}') -> Unix ---")
    converted_ts = TimeConverter.seoul_to_unix(seoul_str)
    print(f"Result: {converted_ts:.2f}")

    # 검증
    diff = abs(current_ts - converted_ts)
    print(f"\n[검증] 오차(초): {diff:.6f} (마이크로초 단위의 파싱 손실은 정상입니다)")

```

### 코드 핵심 포인트 (Expert Note)

1. **Aware vs Naive Datetime**:
* `datetime.strptime`으로 문자열을 파싱하면 타임존 정보가 없는 **Naive Datetime**이 생성됩니다.
* 이 상태에서 바로 `.timestamp()`를 호출하면 시스템 로컬 타임존으로 간주해버릴 위험이 있습니다.
* 반드시 `.replace(tzinfo=SEOUL_TZ)`를 통해 **"이 시간은 서울 시간이다"**라고 명시(Aware Datetime으로 변환)한 뒤 타임스탬프를 추출해야 정확합니다.


2. **`fromtimestamp`의 `tz` 인자**:
* `datetime.fromtimestamp(ts)` 만 쓰면 시스템 로컬 타임존이 적용됩니다.
* `datetime.fromtimestamp(ts, SEOUL_TZ)`와 같이 두 번째 인자를 명시해야 서버의 로컬 설정과 무관하게 항상 서울 시간을 보장합니다.



ROS 2 노드에서 로그를 남기거나, 백엔드 서버와 시간을 동기화할 때 이 클래스를 유틸리티 파일(예: `time_utils.py`)로 저장해두고 사용하시면 편리합니다.

ROS 2의 `builtin_interfaces/Time` 메시지(sec, nanosec)를 바로 이 변환기에 태울 수 있는 래퍼 함수도 필요하신가요?
