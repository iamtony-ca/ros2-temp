**Constrained Smoother**는 매우 강력한 최적화 도구(Ceres Solver 기반)이므로, 파라미터를 잘 만지면 Planner가 만들어낸 거친 K-shape(전진-후진-전진)를 부드러운 U-턴이나 곡선으로 "다림질"할 수 있습니다.

K-shape를 없애기 위한 핵심 전략은 \*\*"원본 경로를 무시하더라도(Low Fidelity), 무조건 부드럽게(High Smoothness) 펴라"\*\*는 명령을 내리는 것입니다.

다음은 K-shape 제거에 특화된 `nav2_params.yaml` 튜닝 가이드입니다.

-----

### 🚀 K-Shape 제거를 위한 핵심 파라미터 튜닝

#### 1\. 주요 전략

1.  **`w_smooth` 극대화**: 경로를 펴려는 힘을 아주 강하게 줍니다.
2.  **`w_dist` 0 유지**: 원본 경로(K-shape가 있는 경로)에 붙어있으려는 성질을 없애야 합니다.
3.  **`path_downsampling_factor` 증가**: 점들이 너무 촘촘하면(Dense) 모양을 바꾸기 어렵습니다. 듬성듬성하게 점을 잡아야 최적화 과정에서 경로의 모양(Topology)이 크게 바뀝니다.
4.  **`max_iterations` 증가**: 경로의 형태를 완전히 바꾸는 것은 어려운 연산이므로 시간을 충분히 줍니다.

#### 2\. 추천 YAML 설정

```yaml
smoother_server:
  ros__parameters:
    smoother_plugins: ["SmoothPath"]
    SmoothPath:
      plugin: "nav2_constrained_smoother/ConstrainedSmoother"
      
      # [중요] 전진/후진 감지 켜기
      # 이걸 꺼버리면(false) Planner가 준 K-turn 경로 자체를 이해 못 하고 에러가 날 수 있습니다.
      # 켜두되(True), 아래 파라미터들로 후진 구간을 없애도록 유도해야 합니다.
      reversing_enabled: true 

      # [핵심 1] 다운샘플링 (매우 중요)
      # 경로의 점을 듬성듬성 잡아야 K 모양을 U 모양으로 펴기 쉽습니다.
      # 기본값 1 -> 3 ~ 5 추천
      path_downsampling_factor: 3
      
      # 업샘플링: 결과물은 다시 부드럽게 채워줍니다.
      path_upsampling_factor: 1

      # [핵심 2] 회전 반경 (물리적 한계)
      # 로봇이 실제로 돌 수 있는 최소 반경입니다.
      # 이 값이 너무 크면(여유를 너무 많이 주면), Smoother는 "이 반경으로는 한 번에 못 돈다"고 판단해
      # K-shape를 유지할 수밖에 없습니다. 실제 스펙에 딱 맞게(타이트하게) 설정하세요.
      minimum_turning_radius: 0.40 

      # [핵심 3] 가중치 튜닝 (K-shape 제거의 열쇠)
      
      # w_smooth: 매끄러움 가중치. 이 값이 지배적이어야 합니다.
      # 기본값 2000000.0 -> 5000000.0 이상으로 매우 강력하게 설정
      w_smooth: 5000000.0
      
      # w_dist: 원본 경로와의 거리 가중치.
      # 0.0으로 설정하여 원본 K-shape 경로에서 마음껏 벗어날 수 있게 허용합니다.
      w_dist: 0.0
      
      # w_cost: 장애물 회피 가중치.
      # 너무 높으면 벽에서 멀어지려고 하다가 다시 중앙으로 복귀하며 K-shape가 생길 수 있습니다.
      # 적당히 낮게 유지 (기본값 0.015 ~ 0.01)
      w_cost: 0.01

      # w_curve: 곡률 제한 가중치.
      # minimum_turning_radius를 지키려는 힘입니다.
      w_curve: 30.0

      # [핵심 4] 최적화 반복 횟수
      # 경로 모양을 뜯어고치는 건 힘든 작업입니다. 횟수를 늘려주세요.
      optimizer:
        max_iterations: 500   # 기본값 70/100 -> 500 (충분한 연산 시간 부여)
        debug_optimizer: false
        gradient_tol: 1e-4    # 약간 완화해도 됨
        fn_tol: 1.0e-15
        param_tol: 1.0e-20
```

-----

### 💡 전문가의 분석: 이것만은 주의하세요

Smoother가 K-shape를 없애는 데는 **물리적인 전제 조건**이 있습니다.

1.  **공간의 너비:**

      * 만약 로봇이 있는 복도의 폭이 `2m`이고, 로봇의 `minimum_turning_radius`가 `1.2m`라면, **물리적으로 U-턴이 불가능**합니다 (지름 2.4m 필요).
      * 이 경우 아무리 `w_smooth`를 높여도 Smoother는 물리 법칙(Turning Radius)을 어길 수 없으므로 K-shape(후진)를 유지하거나, 최적화에 실패합니다.
      * 이때는 `minimum_turning_radius` 값을 조금 속여서(더 작게) 줄여보거나, Planner 단계에서 해결해야 합니다.

2.  **Cusp Multiplier (`w_cost_cusp_multiplier`):**

      * 이 값은 K-turn 지점(Cusp)에서 장애물과 더 멀리 떨어지게 만드는 파라미터입니다.
      * K-shape를 **없애고 싶다면** 이 값이 너무 크면 안 좋습니다. Cusp 지점을 "안전하게 보호"하려 들기 때문입니다. 이 값을 기본값(3.0)이나 그 이하로 유지하세요.

### 🚀 Next Step

위 설정을 적용해 보시고, 혹시 **"Smoother가 경로를 펴긴 했는데 장애물(벽)을 파고들어요"** 같은 현상이 발생하면 말씀해 주세요. 그때는 `w_cost`와 `w_smooth`의 밸런스를 조정하는 법을 알려드리겠습니다.
