안녕하세요! 로보틱스 소프트웨어 개발자로서 Nav2 Jazzy 환경에서 MPPI를 활용해 모바일 매니퓰레이터나 다중 로봇 제어를 하시다 보면, 특히 **협소 구간(Narrow Passage)**에서 발생하는 이 문제는 매우 흔하면서도 까다로운 부분입니다.

제공해주신 `optimizer.cpp` 코드와 파라미터를 분석한 결과, **"Optimizer fail to compute path"**가 발생하는 근본적인 이유는 MPPI가 생성한 `batch_size`(2000개)의 모든 샘플 궤적이 **Critics(제약 조건)**에 의해 유효하지 않다고 판정되어 `fail_flag`가 `true`가 되었기 때문입니다.

특히 협소 구간에서 성능을 개선하기 위한 핵심 포인트들을 짚어드리겠습니다.

---

## 1. 실패의 핵심 원인 분석

`optimizer.cpp`의 `fallback` 함수를 보면:

```cpp
bool Optimizer::fallback(bool fail) {
  if (!fail) return false;
  reset();
  if (++counter > settings_.retry_attempt_limit) {
    throw nav2_core::NoValidControl("Optimizer fail to compute path");
  }
  return true;
}

```

`retry_attempt_limit`이 현재 **1**로 설정되어 있어, 한 번만 실패해도 즉시 `NoValidControl` 예외를 던지며 주행을 포기합니다. 협소 구간에서는 로봇의 풋프린트가 장애물에 아주 살짝만 걸려도 모든 샘플이 탈락할 수 있습니다.

## 2. 파라미터 튜닝 가이드

### A. Optimizer 및 기본 설정 변경

* **`retry_attempt_limit`**: 현재 1에서 **2~3** 정도로 늘려주세요. 한 번의 iteration에서 실패하더라도 다시 샘플링할 기회를 주어야 합니다.
* **`iteration_count`**: 현재 1인데, 협소 구간에서는 **2~3**으로 높이는 것을 권장합니다. MPPI는 반복을 통해 최적의 제어 시퀀스로 수렴하기 때문에, 좁은 길에서는 더 정교한 수렴이 필요합니다.
* **`sampling_std` (가장 중요)**:
* `vx_std: 0.2`, `wz_std: 0.4`는 일반적인 주행에는 적합하나, 좁은 길에서 미세한 조정을 하기에는 노이즈가 너무 클 수 있습니다.
* **`vx_std: 0.1`, `wz_std: 0.3**` 정도로 낮추어 로봇이 더 조심스럽게(fine-grained) 움직이도록 유도하세요.



### B. Critics 최적화 (협소 구간용)

* **`CostCritic`의 `critical_cost**`: 현재 `300.0`입니다. 만약 Costmap의 inflation 레이어 값이 이보다 높게 설정되어 있다면, 로봇이 벽 근처에 가기도 전에 모든 궤적을 "충돌"로 간주합니다.
* 벽에 더 붙어서 통과해야 한다면 이 값을 **가장 치명적인 충돌 직전 값(예: 250~252)**까지 미세하게 조정해 보세요.


* **`PathAlignCritic` vs `CostCritic**`:
* 현재 `PathAlignCritic`의 `cost_weight`가 **14.0**으로 매우 높습니다. 이는 로봇이 경로(Global Path)를 억지로 추종하게 만듭니다.
* 좁은 길에서 Global Path가 약간 벽 쪽에 치우쳐 있다면, 로봇은 벽을 피하려는 `CostCritic`과 경로를 지키려는 `PathAlignCritic` 사이에서 진동하다가 실패합니다.
* **`PathAlignCritic`의 가중치를 8.0~10.0 정도로 낮추고**, `CostCritic`의 가중치를 상대적으로 유지하여 안전을 우선시하게 하세요.



### C. Motion Model 및 Constraints

* **`model_dt`**: 0.05(20Hz)는 적절하지만, 로봇의 가속도 제약(`ax_max: 3.0`)이 상당히 높습니다. 협소 구간 진입 시 속도가 너무 빠르면 MPPI가 제동 거리를 확보하지 못해 실패할 수 있습니다.

---

## 3. 권장 수정 파라미터 (Summary)

```yaml
      # Optimizer Settings
      iteration_count: 2       # 1 -> 2로 증가 (더 정교한 수렴)
      retry_attempt_limit: 2   # 1 -> 2로 증가 (실패 시 재시도)
      
      # Sampling Noise (좁은 길에서 미세 조종)
      vx_std: 0.1              # 0.2 -> 0.1
      wz_std: 0.3              # 0.4 -> 0.3

      # PathHandler
      PathHandler:
        enforce_path_inversion: true 
        inversion_xy_tolerance: 0.1    # 0.05는 너무 타이트할 수 있음
        inversion_yaw_tolerance: 0.1

      # Critics
      CostCritic:
        cost_weight: 4.0
        critical_cost: 252.0   # Inflation 영역을 더 잘 활용하도록 조정
        consider_footprint: true
      
      PathAlignCritic:
        cost_weight: 8.0       # 14.0 -> 8.0 (경로 추종보다 장애물 회피 공간 확보)
        
      # 만약 제자리 회전이 필요하다면
      # PathAngleCritic의 가중치를 조절하여 진입 각도를 먼저 맞추게 유도하세요.

```

## 4. 추가 팁: Isaac Sim 및 nvblox 활용 시

사용자 정보에 **Isaac Sim**과 **nvblox**를 언급하셨는데, 만약 시뮬레이션 환경이라면 `model_dt`와 실제 시뮬레이션의 `physics_dt`가 동기화되어 있는지 확인이 필요합니다. 또한, `nvblox`를 사용 중이라면 Costmap의 업데이트 주기가 MPPI의 연산 주기보다 늦어 로봇이 이미 장애물 안에 있다고 판단되는 경우(`Optimizer reset`)가 잦을 수 있으니 `update_frequency`를 점검해 보시기 바랍니다.

**혹시 특정 구간에서 로봇이 벽에 부딪히며 실패하는 건가요, 아니면 단순히 멈춰서 움직이지 못하는 건가요?** 상황에 따라 `ObstaclesCritic`의 사용 여부를 다시 검토해 드릴 수 있습니다.

##################################  
#####################################  
########################################  
협소 구간 교행과 벽에 바짝 붙는 경로 추종을 위해, `optimizer.cpp` 코드 분석과 제공해주신 파라미터를 바탕으로 **최종 튜닝 가이드**를 정리해 드립니다.

핵심 전략은 **"로봇이 벽 근처(High Cost)에 가는 것을 허용하되, 샘플링 횟수를 늘려 좁은 틈을 찾을 확률을 높이는 것"**입니다.

---

## 1. Optimizer & Sampling (두뇌 회전 속도와 정밀도)

로봇이 멈추지 않고 끝까지 길을 찾게 하려면 샘플링의 질을 높여야 합니다.

* **`iteration_count`: 1 → 3**
* 한 번의 계산으로 답을 못 찾으면, 최적의 제어 시퀀스를 찾을 때까지 3번 반복합니다. 협소 구간에서 수렴성을 크게 높여줍니다.


* **`retry_attempt_limit`: 1 → 5**
* 현재는 한 번만 실패해도 포기(`fail to compute path`)합니다. 재시도 횟수를 늘려 다시 샘플링할 기회를 줍니다.


* **`vx_std`: 0.2 → 0.1 / `wz_std: 0.4 → 0.3**`
* 노이즈가 너무 크면 좁은 길에서 궤적이 사방으로 튀어 모두 장애물에 걸립니다. 표준 편차를 줄여 더 정교한 움직임을 생성하세요.


* **`batch_size`: 2000 (유지)**
* 벽에 붙은 경로를 통과하려면 "우연히" 벽을 안 건드리는 궤적이 나와야 하므로, 샘플 수는 많을수록 유리합니다.



---

## 2. Critics (심리적 마지노선 조정)

벽에 붙은 경로를 '장애물'로 인식하지 않게 만드는 것이 핵심입니다.

* **`CostCritic.critical_cost`: 300.0 → 251.0** (가장 중요)
* Nav2에서 코스트 253은 내접 반경(Inscribed)을 의미합니다. 300은 너무 높아서 이미 충돌한 궤적까지 계산에 포함시킬 수 있고, 반대로 너무 낮으면 벽 근처만 가도 궤적을 버립니다. **251~252** 정도로 설정해 "벽에 닿기 직전까지는 가도 된다"고 허용하세요.


* **`PathAlignCritic.cost_weight`: 14.0 → 20.0 이상**
* 벽에 붙은 Global Path를 '억지로' 따라가게 하려면 이 가중치를 높여야 합니다.


* **`PathAlignCritic.max_path_occupancy_ratio`: 0.05 → 0.15**
* 경로 자체가 벽에 너무 붙어 있어 경로 위의 코스트가 높더라도, MPPI가 그 경로를 유효하다고 판단하게 해줍니다.



---

## 3. PathHandler & Constraints (유연성 확보)

* **`inversion_xy_tolerance`: 0.05 → 0.1**
* 협소 구간에서 미세하게 경로를 벗어났을 때, 갑자기 경로가 반전되었다고 판단해 멈추는 현상을 방지합니다.


* **`vx_min`: -0.35 (유지 또는 조절)**
* 막혔을 때 뒤로 살짝 빼서 각도를 다시 잡을 수 있도록 음수 값을 유지하세요. 만약 후진을 아예 안 하길 원하신다면 0.0으로 두되, 이 경우 협소 구간에서 끼었을 때 탈출이 어렵습니다.



---

## 4. Costmap Inflation (물리적 제약 확인)

이 부분은 `local_costmap` 섹션에서 확인해야 합니다.

* **`inflation_radius` 확인**:
* 만약 로봇 폭이 0.6m(반지름 0.3m)인데 `inflation_radius`가 1.0m라면, 좁은 복도는 코스트맵 상에서 아예 지나갈 수 없는 빨간색 구역으로 꽉 차게 됩니다.
* **권장**: 로봇의 `inscribed_radius` + 최소 여유 공간(5~10cm) 정도로 반경을 타이트하게 잡으세요.



---

## [종합 수정 제안 예시]

```yaml
      # Optimizer 관련
      iteration_count: 3
      batch_size: 2000
      retry_attempt_limit: 5
      vx_std: 0.1
      wz_std: 0.3
      
      # Critics 관련
      CostCritic:
        enabled: true
        cost_weight: 3.8
        critical_cost: 251.0   # 253(Inscribed)에 가깝게 조정
        consider_footprint: true
        
      PathAlignCritic:
        enabled: true
        cost_weight: 20.0      # 벽에 붙은 경로를 강하게 추종
        max_path_occupancy_ratio: 0.15
        
      PathHandler:
        prune_distance: 1.1
        enforce_path_inversion: true
        inversion_xy_tolerance: 0.1
        inversion_yaw_tolerance: 0.1

```

**다음 단계로 무엇을 도와드릴까요?**

1. 위 파라미터를 적용한 후에도 특정 구간에서 멈춘다면, 그 당시의 **Local Costmap 스크린샷**이나 **로봇의 풋프린트 설정값**을 알려주세요.
2. 혹은 `Optimizer reset` 시 발생하는 **상세 에러 코드나 속도 명령(cmd_vel) 값**을 분석해 드릴 수 있습니다.
