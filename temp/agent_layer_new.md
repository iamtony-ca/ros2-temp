여기서는 `multi_agent_nav2::AgentLayer`의 **모든 메서드**를 코드에 기반해 설명합니다. 설명은 한국어로 하되, 기술 용어(예: layer, costmap, footprint, dilation, smear, ROI 등)는 원문 그대로 씁니다. 끝부분에는 **동작 흐름(데이터 플로우)**, **thread-safety**, **성능/튜닝 팁**, **자주 하는 실수**도 정리했습니다.

---

# 개요

`AgentLayer`는 Nav2 `nav2_costmap_2d::Layer`를 상속한 **커스텀 costmap layer**입니다. `/multi_agent_infos`로 들어오는 다른 에이전트들의 상태(현재 pose, truncated_path, footprint, status 등)를 받아, **현재 위치 footprint**와 **truncated_path(경로) 상 footprint의 얇은 샘플**을 **등방성 dilation**(+ 선택적 전방 smear)을 적용해 **master costmap**에 rasterize(래스터화)합니다. 결과적으로 **다른 로봇의 현재 위치 및 진행 경로를 튜브 형태의 cost**로 반영합니다. 필요 시 메타 정보를 `agent_layer_meta`로 내보냅니다.

---

# 클래스 인터페이스

## `class AgentLayer : public nav2_costmap_2d::Layer`

### 생성자

#### `AgentLayer::AgentLayer()`

* **역할:** 기본 생성자. 멤버를 default 값으로 초기화.
* **주의:** 실질 초기화는 `onInitialize()`에서 수행.

---

# Lifecycle 메서드

## `void onInitialize() override`

* **역할:** Layer가 로드될 때 호출. ROS parameters 선언 및 get, QoS/토픽/기본 상태 준비, 내부 상태 초기화(`current_=true`, `matchSize()`).
* **핵심 파라미터 선언/획득:**

  * 활성화 여부: `enabled`
  * 구독 토픽: `topic` (기본 `/multi_agent_infos`)
  * 자기 식별자: `self_machine_id`, `self_type_id`
  * frame 검사: `use_path_header_frame` (경로 header.frame_id와 global frame 일치 요구)
  * ROI 반경: `roi_range_m`
  * freshness timeout: `freshness_timeout_ms` (stale 데이터 무시 기준)
  * 비용: `lethal_cost`, `moving_cost`, `waiting_cost`, `manual_cost_bias`
  * **팽창/스미어:** `dilation_m`(등방성), `forward_smear_m`(전방 +x), `sigma_k`(pos_std_m 가중)
  * 메타 퍼블리시: `publish_meta`, `meta_stride`
  * 경로 샘플 제한: `max_poses`
  * QoS: `qos_reliable`
* **부가:** `matchSize()`로 layer 크기를 master에 맞춤. `enabled_`가 true면 `activate()` 호출.

## `void activate() override`

* **역할:** 실제 구독/퍼블리셔 생성.
* **구독:** `sub_` ← `multi_agent_msgs::msg::MultiAgentInfoArray` (`topic_`, QoS는 reliable/best_effort 선택)
* **퍼블리시(옵션):** `meta_pub_` ← `multi_agent_msgs::msg::AgentLayerMetaArray` (transient_local, reliable)
* **언제 호출?:** 보통 lifecycle 활성화 시점 또는 `onInitialize()` 내에서 `enabled_`가 true일 때.

## `void deactivate() override`

* **역할:** 구독/퍼블리셔 해제. 리소스 정리.
* **효과:** 이후에는 데이터 갱신/출력이 일시 중지됨.

## `void reset() override`

* **역할:** layer를 “현재 최신 상태(current_)”라고 표시.
* **구현:** `current_ = true;`
* **의미:** cost 업데이트 루프에서 “추가 업데이트 필요 없음” 신호로 사용.

---

# Costmap 콜백 (nav2_costmap_2d::Layer 인터페이스)

## `void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override`

* **역할:** 이번 주기에서 **cost를 갱신해야 하는 world bounds**를 master에게 알림.
* **입력:**

  * `robot_x/y/yaw`: 로봇의 현재 pose (여기서는 yaw 미사용)
  * `min_x/min_y/max_x/max_y`: 이번 사이클에서 수정될 수 있는 AABB를 확장하는 in/out 파라미터
* **동작:**

  1. `enabled_`가 아니면 return.
  2. `last_infos_`를 mutex로 보호하여 스냅샷. `stale()` 검사로 timeout된 데이터는 무시.
  3. `global_frame` = layer가 소속된 costmap frame.
  4. 각 agent `a`에 대해:

     * `isSelf(a)`면 skip (자기자신 제외)
     * ROI(거리) 필터: `roi_range_m_` 밖이면 skip
     * 선택적 frame check: `use_path_header_frame_`가 true일 때 `a.truncated_path.header.frame_id != global_frame`면 skip
     * **bounds 갱신:** (a) 현재 pose, (b) truncated_path의 각 pose 위치를 min/max에 반영
  5. 하나라도 반영되었다면 입력 AABB를 out AABB에 merge.
* **성능 팁:** `max_poses_`로 truncated_path 샘플 수를 제한하여 bounds 계산 비용 억제.

## `void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override`

* **역할:** master costmap에 실제 **cost 값을 rasterize**.
* **입력:**

  * `master_grid`: 수정 대상 costmap
  * `min_i/min_j/max_i/max_j`: master가 제안하는 업데이트 영역(여기 구현에서는 직접 쓰지 않음; 전체 rasterize)
* **동작:**

  1. `enabled_`가 아니면 return.
  2. `last_infos_` snapshot + `stale()` 검사 (mutex 보호)
  3. loop: 각 agent `a`(self 제외)에 대해 `rasterizeAgentPath(a, &master_grid, meta_hits)` 호출
  4. `publish_meta_`가 true면, 추출된 `meta_hits`를 `AgentLayerMetaArray` 포맷으로 `meta_pub_`에 publish

     * `meta_stride_` 간격으로 샘플링해 트래픽 감소
* **주의:** 현재 구현은 `min_i..max_i` 등의 영역 제한을 활용하지 않고 **관심 영역만을 updateBounds에서 관리**합니다.

---

# I/O & 데이터 핸들링

## `void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)`

* **역할:** `/multi_agent_infos` 구독 콜백. 최신 배열과 stamp 저장.
* **thread-safety:** `data_mtx_`로 보호.
* **주의:** 여기서는 validation/필터링을 하지 않고 **수신만** 담당. 사용 시점(다른 메서드)에서 stale/ROI/frame 필터를 적용합니다.

## `bool stale(const rclcpp::Time & stamp) const`

* **역할:** 데이터 신선도 검사.
* **기준:** `now - stamp > freshness_timeout_ms_`.
* **의미:** timeout된 에이전트 데이터는 costmap 반영에서 제외하여 **유령 cost**를 방지.

## `bool isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const`

* **역할:** 자기 자신 판단(같은 `machine_id` && `type_id`).
* **의미:** self collision을 layer가 스스로 그리지 않게 예외 처리.

---

# 비용/팽창/상태 유틸

## `unsigned char computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const`

* **역할:** 해당 agent에 대해 rasterize 시 사용할 base cost 계산.
* **로직:**

  * `AgentStatus.phase`가 `STATUS_MOVING` 또는 `STATUS_PATH_SEARCHING`이면 **moving_cost_**, 아니면 **waiting_cost_**.
  * `a.mode == "manual"`이면 `manual_cost_bias_`를 가산 후 0~254 클램프.
* **용도:** `rasterizeAgentPath()`에서 현재 footprint 및 경로 footprint 찍을 때 사용.

## `double computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const`

* **역할:** **등방성 dilation(r)** 반환. `forward smear`는 제외.
* **로직:** `r = dilation_m_` + (if `a.pos_std_m >= 0.0` then `sigma_k_ * a.pos_std_m`)
* **의미:** 위치 불확실도(pos_std_m)를 dilation에 반영해 **보수적으로 튜브 확대**.

## `static inline bool isMovingPhase(uint8_t phase)`

* **역할:** 이동 상태 판정 헬퍼.
* **true 조건:** `STATUS_MOVING` 또는 `STATUS_PATH_SEARCHING`.
* **용도:** 이동 중일 때만 **forward smear**를 적용하도록 제어.

---

# 래스터화 파이프라인

## `static inline std::vector<geometry_msgs::msg::Point> dilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in, double iso_dilate_m, double forward_len_m)`

* **역할:** **로컬 프레임**에서 footprint 다각형을 변형:

  * (1) **등방성 dilation**: centroid 기준으로 모든 vertex를 외측으로 `iso_dilate_m`만큼 확장
  * (2) **전방(+x) smear**: centroid보다 **x가 큰 쪽 vertex만** `forward_len_m`만큼 +x 평행 이동
* **입력:**

  * `in`: 로컬 footprint (Point32)
  * `iso_dilate_m`: 등방성 확장량
  * `forward_len_m`: 전방 smear 길이 (0이면 비적용)
* **출력:** 로컬 좌표계의 `geometry_msgs::msg::Point` 다각형
* **포인트:** “전방”의 정의는 **로컬 footprint의 +x 방향**입니다. 최종 world 변환은 `fillFootprintAt()`에서 pose 기반으로 수행.

## `void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp, const geometry_msgs::msg::Pose & pose, double extra_dilation_m, double forward_len_m, nav2_costmap_2d::Costmap2D * grid, unsigned char cost, std::vector<std::pair<unsigned int,unsigned int>> * meta_hits = nullptr)`

* **역할:** 단일 pose에서 **(등방성 + 전방) 변형된 footprint**를 **world로 변환**하고, 그 **AABB 범위의 grid cell**을 **point-in-polygon(ray casting)**으로 판정해 **Max-merge 방식**으로 cost를 write.
* **단계:**

  1. `dilateFootprintDirectional()`로 로컬 footprint 확장
  2. `pose`(yaw 포함)로 **world 변환**
  3. world 다각형의 bbox를 `worldToMapEnforceBounds`로 map index 범위로 변환
  4. bbox 내부를 raster scan:

     * `mapToWorld(i,j)` → `(wx, wy)`
     * `pointInPolygon(poly, wx, wy)`로 내부 판정
     * 내부면 `grid->getCost(i,j)`와 비교하여 **더 큰 값(cand)이면 setCost** (NO_INFORMATION은 0 취급)
     * `meta_hits`가 있으면 `(i,j)`를 push
  5. 이번 사이클 bounds bookkeeping(내부 필드 `touch_min/max_*`)
* **주의:**

  * **True overwrite** 대신 **Max-merge**를 씀. 기존 장애물보다 낮은 cost로 덮지 않음.
  * 전방 smear는 이 함수에 **forward_len_m**로 직접 전달되어 적용.

## `bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly, double x, double y)`

* **역할:** ray casting에 의한 PIP(point-in-polygon).
* **세부:** 수직 교차 수를 토글 방식으로 계산(짝홀 반전).
* **주의:** 분모 0 보호를 위해 `max(1e-12, (yj-yi))` 사용.

## `void rasterizeAgentPath(const multi_agent_msgs::msg::MultiAgentInfo & a, nav2_costmap_2d::Costmap2D * grid, std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)`

* **역할:** **단일 agent**에 대해 현재 pose와 truncated_path의 pose들을 costmap에 rasterize.
* **로직:**

  * `cost_now = computeCost(a)`
  * `iso_extra = computeDilation(a)`
  * `forward_len = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0`
  * **현재 pose**에 대해: `fillFootprintAt(a.footprint, a.current_pose.pose, iso_extra, forward_len, grid, cost_now, &meta_hits)`
  * **경로 pose들**에 대해(최대 `max_poses_`):

    * 권장치: 등방성은 **소량(예: iso_extra*0.5)**, 전방 smear는 **0.0** (경로 쪽은 과차단 방지)
    * `fillFootprintAt(a.footprint, ps, iso_extra*0.5, 0.0, grid, cost_now, &meta_hits)`
* **포인트:** “현재 위치는 강하게, 경로는 얇게”가 기본 전략. 필요시 주석 부분을 조정하면 경로 쪽도 더 두껍게 가능.

---

# ROS I/F: 메타 퍼블리시

## `meta_pub_` → `multi_agent_msgs::msg::AgentLayerMetaArray`

* **역할:** 이번 사이클에서 실제로 칠해진 cell들의 **샘플**을 내보냄(디버그/시각화/분석용).
* **내용:** 각 cell에 대한 world 좌표, map index 등. (agent 식별자는 여기선 0으로 채움 — 필요 시 확장 가능)
* **간격:** `meta_stride_` 단위로 샘플링해 과도한 트래픽을 피함.

---

# 동작 흐름(데이터 플로우)

1. **입력 수신:** `infosCallback()`가 `/multi_agent_infos`를 수신하여 최신 배열/시간을 저장.
2. **bounds 계산:** `updateBounds()`가 ROI/신선도/frame 필터를 적용해 **이번 사이클에 칠할 가능성이 있는 AABB**를 master에 전달.
3. **cost 갱신:** master가 `updateCosts()`를 호출하면, snapshot한 agent들에 대해 `rasterizeAgentPath()`로

   * 현재 pose footprint를 **강하게** (등방성 + 이동 중이면 전방 smear)
   * truncated_path footprint를 **얇게** (등방성만 소량, 보통 smear=0)
   * Max-merge로 master costmap에 반영.
4. (옵션) **메타 출력:** `agent_layer_meta`로 칠해진 셀들의 샘플을 퍼블리시.

---

# Thread-safety

* `/multi_agent_infos` 수신/소비는 **`data_mtx_`**로 보호.
* costmap API 호출은 Nav2 layer 스레드 컨텍스트 안에서 일련적으로 수행되므로 별도 락은 필요 없음(단, 외부에서 동시 접근하지 않는 것이 전제).

---

# 성능/튜닝 포인트

* **`roi_range_m_`**: 너무 크면 불필요한 범위를 rasterize하여 비용 증가. 실제 상호작용 가능한 거리로 제한.
* **`max_poses_`**: truncated_path 샘플 상한. 클수록 경로 튜브가 길어지지만 계산량 증가.
* **`dilation_m_` / `sigma_k_`**: 튜브 두께. pos_std_m이 보고되는 센서 품질에 맞춰 조정.
* **`forward_smear_m_`**: 이동 중 전방 여유. 좁은 통로에서 상호회피 성능을 끌어올리되 과도하면 과차단.
* **`moving_cost_ / waiting_cost_ / lethal_cost_`**: cost 계층. planner가 우회하도록 충분히 높으나 false positive 시 전체 경로를 막지 않도록 적정치 필요.
* **`freshness_timeout_ms_`**: 통신 지연/로스 환경에서는 너무 짧으면 깜빡임, 너무 길면 고스트 잔상. 실제 메시지 주기+네트워크 상황에 맞춰 조절.

---

# 자주 하는 실수 & 주의사항

* **frame mismatch**: `use_path_header_frame_ == true`인데 에이전트가 다른 frame으로 `truncated_path`를 보내면 전부 skip됩니다. frame 정합 확인 필수.
* **self masking**: `self_machine_id_ / self_type_id_` 설정이 틀리면 **자기 자신의 footprint를 칠해** planner가 자기 자신을 피하려고 합니다.
* **over-smearing**: `forward_smear_m_`를 크게 두면 직진 방향으로 지나치게 넓은 차단이 생겨 dead-end처럼 보일 수 있습니다.
* **meta flooding**: `publish_meta_`를 켠 상태에서 `meta_stride_`가 너무 작으면 대역폭 낭비. 시각화/디버깅 시에만 낮추세요.
* **NO_INFORMATION 처리**: Max-merge에서 NO_INFORMATION은 0으로 취급합니다. unknown 공간을 강하게 밀어올릴 수 있으니 `waiting_cost_`/`moving_cost_`의 설정 의도를 명확히 하세요.

---

# PathValidator와의 연계 관점 (요약)

* `AgentLayer`가 그린 **agent_mask**(ex. `agent_layer/costmap_raw`)는 **튜브형 cost(보통 254)**를 제공합니다.
* 상위 `PathValidatorNode`는

  * **정밀 판정:** `/multi_agent_infos`의 **기하학적** overlap(`whoCoversPoint()`)로 agent를 최종 확정하고 ID/TTC를 산출
  * **보조 힌트:** `agentCellBlockedNear()`로 주변에 agent_mask가 있는지 확인하여 정밀 판정 재시도 가치 판단
* 즉, `AgentLayer`는 **“어디가 agent일 가능성이 높은가”**를 costmap 상에 표현하고, `PathValidatorNode`는 이 정보를 **정밀/의미론적 매핑**(ID/TTC 포함)으로 끌어올립니다.

---

필요하면 위 문서를 README/설계 문서 형식으로 재배치해 드릴게요. 파라미터 테이블이나 기본 추천값 표도 추가할 수 있습니다.
