# UR16e + 커스텀 그리퍼 Isaac Sim 튜토리얼 (재현 가능)

CAD(STEP)와 로봇 URDF에서 출발해, Isaac Sim에서 **커스텀 그리퍼가 달린 UR16e**를 움직이는
시뮬레이션 로봇으로 빌드업한다. 재현·확장 가능하도록 설계 — 다른 그리퍼/UR 모델로 바꿔도
동일한 절차가 적용된다.

- **Isaac Sim:** 5.1.0
- **시나리오:** 조립된 STEP(UR16e + 커스텀 그리퍼)이 있고, **UR16e 팔만** URDF가 있다(그리퍼는 없음).
  목표: 팔을 관절화하고, 커스텀 그리퍼를 플랜지에 리지드로 장착한 뒤, 마우스로 workspace를 탐색.
  (ROS 2 bridge는 이후 단계)
- **전략(방식 A):** 팔 = URDF(관절화), 그리퍼 = CAD 메시를 플랜지에 리지드 고정. UR16e 팔은 표준이라
  URDF 메시를 써도 손실이 없고, *커스텀*인 그리퍼만 CAD에서 가져온다.

---

## 0. 에셋 목록 (`/isaac-sim/test_ws/`)

| 파일 | 역할 |
|---|---|
| `UR16e.step/UR16e.step` | UR16e 팔 CAD (Universal Robots 공식 STEP) |
| `GRP-CPL-062.STEP` | Robotiq UR 커플링/어댑터 (UR ISO 9409-1-50-4-M6) |
| `2f-85-grippers-robotiq/Step/2F85_Opened_20190924.STEP` | Robotiq 2F-85 그리퍼 CAD |
| `UR16e_2F85_assembly.step` | 전체 조립 CAD (팔+커플링+그리퍼) — *시각 참조용* |
| **`gripper_2f85_tool.step`** | **커플링+그리퍼를 사전 정렬: 마운트면이 원점, +Z=접근축** — 플랜지 부착에 사용 |
| `ur16e.urdf` | UR16e 단독 URDF, 6 revolute 관절, 절대 메시 경로(바로 임포트 가능) |
| `ur16e_2f85.usd` | 조립 CAD의 정지 USD (관절 없음) — 형상/참조용 |
| `ur16e_gripper_base.usd` | URDF 임포트한 관절 팔 (단계 1 산출물) |
| `ur16e_2f85_rigged.usd` | 팔 + 그리퍼(리지드, tool0) (단계 2 산출물) |

> `gripper_2f85_tool.step`이 핵심 재사용 에셋: **마운트면이 (0,0,0), 그리퍼가 +Z로 뻗도록**
> 정렬돼 있어 UR `tool0` 프레임에 **identity(오프셋 0)** 로 얹힌다. 다른 그리퍼면 이 규약대로
> 다시 만들면 단계 2가 동일하게 재현된다.

---

## 1. 로드맵

| 단계 | 내용 | 산출물 |
|---|---|---|
| 0 | 조립 정지 CAD (바닥에 기립) | `ur16e_2f85.usd` |
| **1** | UR16e URDF 임포트 → **관절 팔** | `ur16e_gripper_base.usd` |
| **2** | 커스텀 그리퍼를 `tool0`에 리지드 부착 | `ur16e_2f85_rigged.usd` |
| 3 | 관절 드래그(마우스)로 workspace 검증 | — |
| **4a** | 2F-85 손가락 관절 추가 → 개폐 | `ur16e_2f85_actuated.usd` |
| 4b | ROS 2 bridge 연동 | — |

---

## 2. URDF 임포터 옵션 (이 경우 추천값)

목표 = **테이블 고정 팔로 workspace 탐색**(아직 접촉작업 없음).
메뉴: `File > Import…` → `/isaac-sim/test_ws/ur16e.urdf` 선택.

| 옵션 | 설정 | 이유 |
|---|---|---|
| **Model** | `Create In Stage` | 로봇 프림을 스테이지에 바로 생성 → 편집·그리퍼 부착이 쉬움. (`Reference`는 별파일 참조, 여러 씬에서 재사용할 때) |
| **USD Output** | 기본 경로 | `Reference` 모드에서만 의미. |
| **Links → Base** | **`Static Base`** | UR은 테이블 고정 → 베이스를 월드에 고정(안 쓰러짐). (`Moveable Base`=자유부유) |
| **Default Density** | 기본(~1000) | fallback일 뿐. UR URDF에 관성값 있음. workspace엔 무관. |
| **Joints & Drives → Ignore Mimic** | 기본(끔) | 팔 단독엔 mimic 관절 없음. (그리퍼 임포트 때만 의미) |
| **Joint Config (Stiffness / Natural Frequency)** | 기본값 | 위치드라이브 게인. 기본이면 목표각 추종이 부드러움. 더 단단히 원하면 stiffness↑ |
| **Drive Type** | **`Acceleration`** | 질량 정규화 → 튜닝 쉽고 안정적. (`Force`=실토크, 고급) |
| **Collision From Visuals** | **끔** | UR URDF에 collision 메시(`.stl`)가 있음 → 그걸 사용. |
| **Collider Type** | **`Convex Hull`** | 가볍고 충분(접촉작업 아직 없음). 오목 정밀충돌은 나중에 `Convex Decomposition`. |
| **All Self-Collision** | **끔** | 켜면 초기 침투로 폭주·느려짐. workspace엔 불필요. 나중에 필요시 켬. |
| **Replace Cylinders with Capsules** | 기본 | UR은 메시 기반이라 영향 미미. |
| **Merge Fixed Joints** | **끔** ⚠️ | `flange`/`tool0`을 별도 프림으로 유지 → 그리퍼 부착 프레임(`tool0`)이 깔끔하게 남음. 켜면 `wrist_3_link`로 합쳐져 플랜지 오프셋을 수동 계산해야 함. |

**한 줄 요약:** `Create In Stage` · `Static Base` · `Acceleration` · `Convex Hull` ·
Self-Collision **끔** · Collision-from-visuals **끔** · Merge-Fixed-Joints **끔**, 나머지 기본.

---

## 3. 단계 1 — 팔 관절화

1. `File > Import…` → `/isaac-sim/test_ws/ur16e.urdf`, 위 옵션으로 **Import**.
   - articulation 루트가 `/World/ur16e`(URDF `robot name="ur16e"`)로 생성되고,
     링크 `base_link … wrist_3_link`, `flange`, `tool0` 포함.
2. **Play (▶)**. Static Base + 위치드라이브라 자세 유지.
3. `Tools > Physics > Physics Inspector` (Isaac 5.x — `Window`에서 여기로 이동) → 로봇(`/World/ur16e`)
   선택 → Play 중 관절 `Target Position` 슬라이더 드래그:
   `shoulder_pan`(베이스 회전), `shoulder_lift`, `elbow`, `wrist_1/2/3`.
   `tool0` 프레임이 **도달 가능 workspace**를 그림.
   - `Tools`에 없으면 `Window > Extensions`에서 `omni.physx.supportui` 활성화, 또는
     `Window > Physics > Physics Authoring Toolbar`도 진입점.
4. **Stop (■)** 후 `File > Save As…` → `/isaac-sim/test_ws/ur16e_gripper_base.usd`.

✅ 체크포인트: 맨 팔이 마우스로 움직이고 자세를 유지.

---

## 4. 단계 2 — 커스텀 그리퍼를 `tool0`에 리지드 부착

그리퍼는 URDF가 없으므로 CAD 형상을 플랜지 링크에 고정한다.

### 4a. 사전 정렬된 그리퍼 툴을 USD로 변환
1. `File > Import…` → `/isaac-sim/test_ws/gripper_2f85_tool.step`
   (Isaac Sim CAD 변환기 `omni.kit.converter.cad` 사용).
2. 일반 메시(비관절 Xform 트리)로 임포트 → `gripper_2f85_tool.usd`로 저장.
   - CAD는 **mm**, 변환기가 `unitsResolve` 스케일(0.001)을 붙여 미터 스테이지에서 올바른 크기로 보임. **이 스케일 유지.**

### 4b. 플랜지 링크 아래로 페어런팅
1. **Stage** 패널에서 임포트된 `gripper_2f85_tool` Xform을 `/World/ur16e/wrist_3_link/flange/tool0`
   **자식으로** 드래그.
2. 그리퍼 Xform의 **로컬 트랜스폼을 identity**로: `translate=(0,0,0)`, `orient=identity`,
   `scale=1` — 단 **`unitsResolve`(0.001) 스케일은 그대로 둠**.
   - 에셋의 마운트면이 CAD 원점·+Z=접근축이고, `tool0`의 +Z가 플랜지 바깥을 향하므로,
     identity면 커플링이 플랜지에 밀착되고 그리퍼가 바깥을 향함. 수동 오프셋 불필요.

### 4c. 링크의 일부로 만들기 (떨어지는 별도 강체가 아니라 리지드)
- 그리퍼 메시는 이제 `tool0` **링크**(articulation의 rigid body)의 자식 → 플랜지와 함께 강체로 움직임.
- 그리퍼에 **별도 `RigidBodyAPI`를 주지 말 것**(자유 강체가 되어 떨어짐). 충돌이 필요하면
  그리퍼 메시에 **Collider**(`Convex Hull`)만 추가.

✅ 체크포인트: Stage 트리에서 그리퍼가 `tool0` 아래; 뷰포트에서 플랜지에 바깥 방향으로 장착.

`File > Save As…` → `/isaac-sim/test_ws/ur16e_2f85_rigged.usd`.

### 4d. 헤드리스 대안 (레퍼런스 결과 생성에 사용)
GUI 대신, `gripper_2f85_tool.step`을 OpenCASCADE로 테셀레이션(미터 단위 삼각망)해
`/ur16e/wrist_3_link/flange/tool0/Gripper2F85`에 **identity**로 스크립트 부착할 수 있다.
동일한 `ur16e_2f85_rigged.usd`가 나온다. 검증된 배치: `tool0` 월드 `(0.838, 0.291, 0.061)`,
그리퍼 마운트면이 `tool0`에 일치, `tool0`의 +Z로 ~0.16 m 뻗음.

> **무해한 로드 경고:** `Unresolved reference … /visuals/base_link`. UR URDF는 베이스를
> `base_link`(질량 프레임)과 `base_link_inertia`(메시 보유)로 나눔. 베이스 비주얼은
> `/visuals/base_link_inertia`에 있어 로봇은 정상 렌더링됨. 무시해도 됨.

---

## 5. 단계 3 — 그리퍼 포함 workspace 검증

1. `ur16e_2f85_rigged.usd` 열기 → **Play (▶)** → `Tools > Physics > Physics Inspector`에서 팔 관절 드래그.
2. 그리퍼가 플랜지를 타고 움직이며 **그리퍼 끝**이 workspace를 스윕.
3. 점검: 모든 관절 동작에서 그리퍼가 플랜지에 고정 유지; 떨어지거나 폭주 없음(Self-Collision 끔).

---

## 6. 단계 4a — 2F-85 손가락 관절 (개폐) — *다음 단계 (예정)*

리지드 부착된 그리퍼는 단일 메시라 그대로는 못 움직인다. 개폐하려면 그리퍼를
**분리된 강체 링크 + revolute 관절**로 리깅해야 한다. (URDF가 없는 시나리오이므로 CAD에서 수동 리깅)

**CAD 분할 사전조사 (`2F85_Opened.STEP`, 솔리드 11개, 좌표 mm):**
- 그리퍼는 **X=0 평면 기준 좌우 대칭**으로 개폐 (손가락이 ±X로 벌어짐).
- **베이스**: 중앙 큰 솔리드 `cx≈0, cy≈32` (226 cm³, 본체).
- **좌 손가락 그룹**: `cx<0` 솔리드들 (knuckle/finger/inner-knuckle/tip, cx ≈ -42·-65·-32·-58·-48).
- **우 손가락 그룹**: `cx>0` 솔리드들 (좌측의 거울상).
- 실제 2F-85는 **4절 링크(닫힌 루프)** 라 정밀 모델은 mimic 관절이 필요. 연습용 단순화:
  **베이스 + 좌/우 손가락 = 강체 3개**, knuckle 피벗에 **revolute 2개**(1 driver + 1 미러),
  팔 articulation 트리에 가지로 추가.

**예정 절차:**
1. `2F85_Opened.STEP`를 base / left / right 그룹별로 분리 테셀레이션 → 강체 3개.
2. 각 손가락의 knuckle 피벗축(위치·방향)을 CAD에서 추출.
3. `tool0` 아래 base 고정 + 좌/우 finger에 revolute joint + 드라이브(개폐 target).
4. Play → Physics Inspector로 `finger_joint` 드래그 → 개폐 확인.
→ 산출물: `ur16e_2f85_actuated.usd`.

---

## 7. 확장

- **다른 그리퍼/커플링:** 같은 규약(마운트면=원점, +Z=접근축)으로 `gripper_2f85_tool.step`을
  다시 만들면 단계 2가 동일.
- **다른 UR 모델:** `ur16e.urdf`를 `ur5e`/`ur10e` 등으로 교체. 플랜지 규격
  (ISO 9409-1-50-4-M6) 공통이라 동일 그리퍼 장착.
- **ROS 2 bridge:** 리깅된 로봇에 articulation/joint-state 퍼블리셔·서브스크라이버(OmniGraph) 추가.

## 8. 재현성 노트

- Isaac Sim **5.1.0**; UR16e URDF는
  `isaacsim.robot_motion.motion_generation/.../universal_robots/ur16e/ur16e.urdf`에서
  `package://ur_description/` → `file:///opt/ros/jazzy/share/ur_description/` 치환(메시 14개 전부 해결).
- 그리퍼 마운트 변환은 `gripper_2f85_tool.step`에 베이크됨(커플링 바닥→원점, +Z=접근축) →
  플랜지 부착이 `tool0` 아래 **identity** 결정론적 배치.
- 모든 STEP은 mm(AP214); Isaac CAD 변환기가 임포트 시 mm→m `unitsResolve` 스케일 적용.
