# Isaac Sim 강의안 — 모듈 2 ~ 7 상세

> 러닝 예제: `ur-isaacsim` (Isaac Sim 5.1 / UR16e + Robotiq 2F-85 + RealSense D405 / ROS2 Jazzy)
> 각 모듈 구성: **학습 목표 → 사전 지식 → 강의 본문(개념 + repo 코드 해설) → 실습(Hands-on) → 자주 만나는 함정 → 정리 체크리스트**

---

# 모듈 2. USD 기초 — Universal Scene Description

## 학습 목표
- USD가 "씬을 기술하는 공용 포맷"이며 Isaac Sim의 모든 것이 USD Stage 위에 올라간다는 점을 이해한다.
- Stage / Prim / Attribute / Relationship / Schema의 관계를 설명할 수 있다.
- 단위(meters-per-unit)와 up-axis가 왜 로봇 자산에서 결정적인지 안다.
- Python(pxr)으로 Stage를 열고 Prim을 만들고 속성을 읽을 수 있다.

## 사전 지식
- 3D 좌표계(위치/회전), 파이썬 기초.

## 강의 본문

### 2.1 USD란 무엇인가
USD(Universal Scene Description)는 Pixar가 만든 오픈소스 3D 씬 기술 포맷이자 API다. Isaac Sim/Omniverse에서 로봇·환경·센서·물리·재질이 **전부 USD로 표현**된다. 즉 "Isaac Sim을 다룬다 = USD Stage를 다룬다"에 가깝다.

핵심 개념 4가지:
- **Stage**: 하나의 씬 전체(메모리에 로드된 USD). `.usd` 파일을 열면 Stage가 된다.
- **Prim**(primitive): Stage 안의 노드. 로봇, 링크, 카메라, 메쉬, 조인트가 모두 Prim. 경로(`/UR16e/wrist_3_link`)로 식별.
- **Attribute**: Prim이 가진 값(위치, 반지름, 색 등). 시간축(timeSamples)도 가질 수 있음.
- **Relationship**: Prim 간 참조 관계(예: 물리 조인트가 연결하는 두 body).

파일 형식:
- `.usda` — 텍스트(ASCII). 사람이 읽고 디버깅하기 좋음. (repo의 `robotiq_2f85_fixed.usda`)
- `.usdc` — 바이너리(crate). 빠르고 작음.
- `.usd` — 위 둘 중 무엇이든 될 수 있는 확장자.

> 🔎 **repo 연결**: `isaac/assets/`에 `.usd`(바이너리 합성 결과)와 `.usda`(텍스트로 보존한 전처리 그리퍼)가 공존한다. "왜 어떤 건 usda이고 어떤 건 usd인가?"를 이 차이로 설명한다.

### 2.2 Xform과 Transform 스택, 단위
로봇 자산에서 가장 많이 실수하는 두 가지가 **단위**와 **up-axis**다. Isaac Sim은 미터(meter), Z-up을 쓴다. 메쉬 변환기는 보통 cm 기본값이라 100배 스케일 사고가 난다.

repo의 변환 스크립트가 이 점을 정확히 못박는다:

```python
# isaac/common/convert_dae_to_usd.py
ctx = asset_converter.AssetConverterContext()
ctx.use_meter_as_world_unit = True   # ★ cm 기본값이면 100배로 커진다
```

합성 스크립트에서도 Stage 생성 직후 단위/축을 명시한다:

```python
# isaac/common/build_ur16e_2f85.py  (Step 2)
out = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(out, 1.0)        # 1 unit = 1 meter
UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z)  # Z-up
```

- **Xformable / Xform**: Prim에 translate/rotate/scale을 거는 인터페이스. 여러 변환이 **순서(XformOpOrder)**대로 쌓여 최종 포즈가 된다.
- `ClearXformOpOrder()` 후 `AddTranslateOp()`, `AddScaleOp()`처럼 명시적으로 쌓는 패턴을 repo 곳곳에서 본다(카메라 바디, 장애물 박스 등).

### 2.3 Schema — Typed vs Applied
USD의 "타입 시스템". Prim이 무엇이고 어떤 속성을 갖는지 정의한다.
- **Typed schema**: Prim의 종류 자체. 예) `UsdGeom.Camera`, `UsdGeom.Cube`, `UsdGeom.Mesh`, `UsdGeom.Xform`.
- **Applied schema(API schema)**: 기존 Prim에 "기능"을 덧붙임. 예) `UsdPhysics.RigidBodyAPI`, `UsdPhysics.ArticulationRootAPI`, `UsdPhysics.DriveAPI`.

repo에서 Applied schema를 조회/판별하는 실제 코드:

```python
# build_ur16e_2f85.py
if "PhysicsArticulationRootAPI" in prim.GetAppliedSchemas():
    ...
if prim.IsA(UsdPhysics.Joint):   # Typed 판별
    ...
```

이 구분은 모듈 3~4에서 "그리퍼의 ArticulationRootAPI를 제거"하거나 "조인트에 DriveAPI를 적용"할 때 핵심이 된다.

### 2.4 Python으로 USD 다루기
Isaac Sim 번들 파이썬(`/isaac-sim/python.sh`)으로 실행한다. 가장 기본 패턴:

```python
from pxr import Usd, UsdGeom

stage = Usd.Stage.Open("some.usd")      # 또는 CreateInMemory()
for prim in stage.Traverse():           # 모든 Prim 순회
    print(prim.GetPath(), prim.GetTypeName(), prim.GetAppliedSchemas())
```

> 강의 팁: `Traverse()`로 Stage를 훑어 트리를 출력해보는 것이 USD를 "손에 익히는" 첫걸음이다. repo의 build 스크립트 마지막 *sanity report*가 정확히 이 방식으로 검증한다(아래 인용).

```python
# build_ur16e_2f85.py 끝부분: 합성 결과 자가검증
n_art   = sum(1 for p in chk.Traverse() if "PhysicsArticulationRootAPI" in p.GetAppliedSchemas())
n_joint = sum(1 for p in chk.Traverse() if p.IsA(UsdPhysics.Joint))
info(f"ArticulationRoot={n_art} (expect 1) ... Joint={n_joint}")
```

## 실습 (Hands-on)
1. `convert_dae_to_usd.py`로 `meshes/picknik_ur5_realsense_camera_adapter_rev2.dae`를 USD로 변환한다.
   - `use_meter_as_world_unit`을 `False`로 바꿔보고 뷰포트에서 100배로 커지는 현상을 직접 확인 → 다시 `True`로.
2. 변환 결과 `.usd`를 `Usd.Stage.Open`으로 열고 `Traverse()`로 Prim 트리를 출력한다.
3. `assets/robotiq_2f85_fixed.usda`를 **텍스트 에디터로 열어** prim 계층/스키마를 눈으로 읽는다.

## 자주 만나는 함정
- 단위 불일치(cm↔m)로 자산이 100배/0.01배가 됨 → 항상 meters 확인.
- up-axis 불일치(Y-up 자산을 Z-up 씬에) → 로봇이 누워서 로드됨.
- `.usdc`(바이너리)를 텍스트로 열려다 실패 → `.usda`로 export 후 확인.

## 정리 체크리스트
- [ ] Stage/Prim/Attribute/Relationship를 한 문장씩 설명할 수 있다.
- [ ] Typed vs Applied schema 예시를 든다.
- [ ] meters/Z-up을 코드로 설정할 수 있다.

---

# 모듈 3. USD 컴포지션 — 참조·레이어·합성으로 로봇 만들기

## 학습 목표
- 컴포지션 아크(References/Payloads/Sublayers/Variants)의 역할을 구분한다.
- 외부 USD 자산을 참조해 내 씬에 가져온다.
- "그대로 참조하면 왜 깨지는가"를 이해하고, 전처리(flatten/instanceable 해제/스키마 제거)를 적용한다.
- 두 자산을 합쳐 **하나의 아티큘레이션**으로 만드는 전체 흐름을 따라 한다.

## 강의 본문

### 3.1 컴포지션 아크 개요
USD의 강력함은 "여러 레이어를 비파괴적으로 합성"하는 데 있다.
- **Reference**: 다른 USD를 현재 Prim 아래로 끌어와 인스턴스화. (가장 많이 씀)
- **Payload**: 무거운 자산을 지연 로드(load/unload 가능한 reference).
- **Sublayer**: 레이어를 통째로 겹쳐 쌓음(상위 레이어가 하위를 오버라이드).
- **Variant**: 한 Prim에 여러 형상/구성 옵션을 담아 스위치(예: 그리퍼 유/무).

### 3.2 외부 자산 참조하기
이 repo는 UR16e와 2F-85를 NVIDIA Omniverse CDN에서 직접 참조한다:

```python
# build_ur16e_2f85.py
UR16E_URL = ".../Isaac/5.1/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd"
EE_URL    = ".../Isaac/5.1/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd"

root = out.DefinePrim("/UR16e", "Xform")
out.SetDefaultPrim(root)
root.GetReferences().AddReference(UR16E_URL)   # ★ UR16e를 /UR16e 아래로 참조
```

`DefinePrim` → `AddReference` 패턴이 컴포지션의 기본 동작이다. `SetDefaultPrim`은 "이 Stage를 다른 곳에서 참조할 때 진입점"을 지정한다.

### 3.3 왜 그대로 참조하면 깨지는가 — 전처리(Step 1)
스크립트 헤더가 문제를 정확히 기술한다(강의에서 그대로 읽어주면 좋다):

> "Robotiq 2F-85 USD는 자기 자신이 하나의 articulation으로 출하된다. 팔의 손목 아래에 그대로 참조하면 (자기 ArticulationRootAPI, instanceable 비주얼 때문에) 실패한다. 그래서 한 번 베이크해서 고친다."

전처리 3단계 코드:

```python
# build_ur16e_2f85.py — Step 1: fix EE
src = Usd.Stage.Open(EE_URL)
work_layer = src.Flatten()           # ① 합성 아크를 평탄화해 단일 레이어로
work = Usd.Stage.Open(work_layer)

for prim in work.Traverse():         # ② instanceable 해제(편집 가능하게)
    if prim.IsInstanceable():
        prim.SetInstanceable(False)

for prim in work.Traverse():         # ③ 그리퍼 자체의 ArticulationRoot 제거
    if "PhysicsArticulationRootAPI" in prim.GetAppliedSchemas():
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

for prim in work.Traverse():         # ④ rigid body의 xform 스택 리셋
    if "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas():
        UsdGeom.Xformable(prim).SetResetXformStack(True)

work.GetRootLayer().Export(str(EE_FIXED))   # -> assets/robotiq_2f85_fixed.usda
```

- **flatten**: 참조/서브레이어를 다 풀어 하나의 레이어로 → 자유롭게 편집 가능.
- **instanceable 해제**: instance 프로토타입은 직접 편집 불가 → 끈다.
- **ArticulationRootAPI 제거**: 합칠 때 articulation root는 **팔의 base 하나여야** 한다. 그리퍼가 자기 root를 들고 오면 충돌.
- **resetXformStack**: 부모(손목) 변환이 자식(그리퍼 강체)에 이중 적용되지 않도록.

### 3.4 합성과 평탄화(Step 2)
전처리한 그리퍼를 손목 아래에 참조하고, 최종 결과를 다시 flatten해서 배포용 단일 파일로 export한다:

```python
gripper_prim = out.DefinePrim("/UR16e/wrist_3_link/gripper", "Xform")
gripper_prim.GetReferences().AddReference(str(EE_FIXED))   # 고친 그리퍼 참조
...
flat_layer = out.Flatten()                 # 합성 결과 평탄화
flat.GetRootLayer().Export(str(COMPOSED))  # -> assets/ur16e_with_2f85.usd
```

**트레이드오프**(강의 핵심 메시지):
- 합성 상태 유지 → 원본 자산 갱신이 자동 반영, 레이어 분리로 협업/오버라이드 유리.
- flatten 후 배포 → 외부 의존성/네트워크 없이 단일 파일, 재현성↑. 대신 레이어 구조는 잃음.
- 이 repo는 "수업·재현용"이라 **flatten된 단일 USD를 배포**한다.

## 실습 (Hands-on)
1. `build_ur16e_2f85.py`를 그대로 실행해 `robotiq_2f85_fixed.usda` + `ur16e_with_2f85.usd`를 생성한다.
2. ④의 전처리 루프 중 하나(예: ArticulationRootAPI 제거)를 주석 처리하고 다시 실행 → sanity report의 `ArticulationRoot=` 값이 1이 아니게 되는 것을 관찰.
3. 생성된 `robotiq_2f85_fixed.usda`(텍스트)를 열어 `references`/`instanceable`/스키마가 어떻게 바뀌었는지 확인.

## 자주 만나는 함정
- articulation root가 2개(팔+그리퍼) → 로봇이 안 움직이거나 분리됨. (sanity report로 `expect 1` 확인)
- instanceable을 안 끄고 편집 시도 → 변경이 무시됨.
- flatten 안 하고 배포 → CDN/네트워크 끊기면 자산 로드 실패.

## 정리 체크리스트
- [ ] Reference와 Sublayer를 구분한다.
- [ ] "왜 전처리가 필요한가"를 ArticulationRoot 관점에서 설명한다.
- [ ] 합성 vs flatten 트레이드오프를 든다.

---

# 모듈 4. USD 물리 & 아티큘레이션

## 학습 목표
- RigidBody/Collision/Mass, Joint(Fixed/Revolute), ArticulationRoot의 의미를 안다.
- FixedJoint로 두 자산을 물리적으로 결합한다(그리퍼 장착).
- Mimic Joint로 1개 구동 → 다수 수동 조인트를 따라가게 한다.
- Drive(stiffness/damping)가 조인트 거동에 미치는 영향을 이해한다.

## 강의 본문

### 4.1 물리 스키마의 구성요소
- **RigidBodyAPI**: 이 Prim은 강체(질량·관성·속도를 가짐).
- **CollisionAPI**: 충돌 형상.
- **MassAPI**: 질량/관성 지정.
- **ArticulationRootAPI**: 관절로 연결된 강체들의 트리(=로봇)의 **루트**. 솔버가 여기서부터 관절 체인을 푼다.

> Isaac UR16e USD는 default prim이 없고, **고정 base 조인트에 ArticulationRootAPI**가 붙어있다. 그래서 로더가 articulation root를 자식 경로로 잡는다:

```python
# ur16e_isaac_ros2.py
# "The Isaac UR16e USD ... applies ArticulationRootAPI to the fixed base joint,
#  so the articulation root is a child prim, not ROBOT_PRIM."
ARTICULATION_ROOT = args.articulation_root or (ROBOT_PRIM + "/root_joint")
```

이 한 줄이 모듈 7(ArticulationController가 어느 Prim을 구동하나)과 직결된다.

### 4.2 Joint 종류와 ArticulationRoot
- **FixedJoint**: 두 body를 강체 결합(상대 운동 0).
- **Revolute/Continuous**: 1축 회전 관절.
- ArticulationRoot 아래의 모든 관절이 하나의 솔버 체인 = "한 로봇".

### 4.3 그리퍼 결합 — FixedJoint 저작
손목(`wrist_3_link`)과 그리퍼 `base_link`를 FixedJoint로 묶어 **6축 팔 + 1축 그리퍼를 한 articulation**으로 만든다:

```python
# build_ur16e_2f85.py
fj = UsdPhysics.FixedJoint.Define(out, ".../gripper_fixed_joint")
fj.CreateBody0Rel().SetTargets([str(wrist_path)])        # 부모: 손목
fj.CreateBody1Rel().SetTargets([str(base_link_path)])    # 자식: 그리퍼 base
fj.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, gripper_z))  # 툴축(+Z) 스탠드오프
fj.CreateLocalRot0Attr().Set(Gf.Quatf(0.7071, (0,0,-0.7071)))  # Z축 -90°
```

- `Body0/Body1 Rel`은 **Relationship**(모듈 2 개념)으로 두 body를 연결.
- `LocalPos0/LocalRot0`로 장착 오프셋(쿼터니언 -90°, 커플링 두께만큼 Z offset)을 준다. → 이게 실제 GRP-ES-CPL-077 커플링 11mm를 표현.

### 4.4 Mimic Joint — 1 구동, 5 추종
2F-85는 물리적으로 1 자유도다. ROS는 `finger_joint` 하나만 명령하고, 나머지 5개는 따라간다. USD에서는 `PhysxMimicJointAPI`가 자산에 베이크되어 있어 sanity report로 개수만 확인한다:

```python
n_mimic = sum(1 for p in chk.Traverse() for s in p.GetAppliedSchemas() if "Mimic" in s)
```

URDF 쪽(모듈 5)에서는 같은 개념이 `<mimic>` 태그로 나타난다(아래 모듈 5에서 대조).

### 4.5 Drive 튜닝
자산 기본 드라이브가 너무 물러서 위치 명령에 굼뜨다. 강체 결합 후 `finger_joint`의 angular DriveAPI를 적용/강화한다:

```python
# build_ur16e_2f85.py
drive = UsdPhysics.DriveAPI.Apply(fjoint, "angular")
drive.CreateStiffnessAttr().Set(20.0)   # 기본 3.0 -> 20.0 (또렷한 개폐)
drive.CreateDampingAttr().Set(1.0)
drive.CreateTargetPositionAttr().Set(0.0)
```

- **Stiffness**: 목표 위치로 끌어당기는 강성(P 게인 느낌).
- **Damping**: 속도 감쇠(D 게인 느낌). 너무 낮으면 진동, 너무 높으면 굼뜸.

## 실습 (Hands-on)
1. 합성된 `ur16e_with_2f85.usd`를 Isaac Sim에 로드하고 `finger_joint`에 위치 목표를 줘서 개폐 확인.
2. `DRIVE_STIFFNESS`를 3.0(원래값)으로 낮춰 다시 baking → 개폐가 느려짐을 비교.
3. `gripper_z`(커플링 오프셋)를 0 / 0.011 / 0.018로 바꿔 베이크하고 RViz/Isaac에서 그리퍼 위치가 달라짐을 관찰.

## 자주 만나는 함정
- FixedJoint의 Body0/Body1을 반대로 → 그리퍼가 엉뚱한 위치.
- 런타임에 그리퍼 Prim의 Xform을 옮겨 위치를 맞추려다 articulation이 깨짐. repo 주석이 명시: **오프셋은 런타임이 아니라 asset에 baking** 하라.

```python
# ur16e_isaac_ros2.py 주석
# "shifting the baked 2F-85 via the fixed joint / prim Xform desynchronises the
#  articulation (broken robot or a dead finger_joint drive). ... BAKE ... and re-export"
```

## 정리 체크리스트
- [ ] ArticulationRoot가 "로봇 하나"의 경계임을 설명한다.
- [ ] FixedJoint로 자산을 결합하고 오프셋을 줄 수 있다.
- [ ] Mimic/Drive의 역할을 구분한다.

---

# 모듈 5. URDF/Xacro & Isaac Sim — 로봇 기술의 두 세계

## 학습 목표
- URDF/Xacro 구조(link/joint/visual/collision/macro)를 읽는다.
- ROS 진영의 URDF와 Isaac의 USD가 **같은 로봇을 두 언어로** 기술함을 이해한다.
- visual/collision 분리, mimic, SRDF 충돌 비활성의 의미를 안다.
- URDF↔USD 정합 시 좌표/단위/조인트 함정을 피한다.

## 강의 본문

### 5.1 URDF/Xacro 기본
- **URDF**: 링크(강체)와 조인트(연결)로 로봇을 기술하는 ROS XML.
- **Xacro**: URDF에 매크로/변수/조건을 넣은 전처리 매크로 언어. 재사용·조립에 필수.

### 5.2 매크로로 조립 — 2F-85 예시
repo의 그리퍼 매크로는 "공식 robotiq_description 메쉬를 쓰되, 마스터 조인트를 `finger_joint`로 바꿔" ros2_control/Isaac이 발행하는 값으로 RViz가 곧장 애니메이션되게 한다:

```xml
<!-- urdf/common/robotiq_2f85_macro.xacro -->
<xacro:macro name="robotiq_2f85" params="prefix parent *origin collision:=true">
  ...
  <!-- MASTER 구동 조인트 (== ros2_control / Isaac finger_joint) -->
  <joint name="${prefix}finger_joint" type="revolute">
    <parent link="${prefix}robotiq_85_base_link"/>
    <child  link="${prefix}robotiq_85_left_knuckle_link"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0.0" upper="0.8" velocity="0.5" effort="50"/>
  </joint>
```

### 5.3 visual vs collision, 그리고 mimic
- visual(.dae)은 항상, collision(.stl)은 `collision:=true`일 때만 방출 → MoveIt 충돌 계획용/시각 전용을 토글:

```xml
<visual><geometry><mesh filename=".../visual/${mesh}.dae"/></geometry></visual>
<xacro:if value="${collision}">
  <collision><geometry><mesh filename=".../collision/${mesh}.stl"/></geometry></collision>
</xacro:if>
```

- **mimic**(URDF판 모듈 4 개념): 나머지 조인트가 `finger_joint`를 배수로 추종:

```xml
<joint name="${prefix}robotiq_85_right_knuckle_joint" type="revolute">
  <mimic joint="${prefix}finger_joint" multiplier="-1"/>
</joint>
```

> 교육 포인트: 모듈 4의 USD `PhysxMimicJointAPI`와 여기 URDF `<mimic>`가 **같은 물리 현실의 두 표현**임을 나란히 보여준다.

### 5.4 Isaac 백엔드를 위한 ros2_control 블록
Isaac과 ros2_control을 토픽으로 잇는 하드웨어 인터페이스가 URDF 안에 선언된다(모듈 7·8의 다리):

```xml
<!-- urdf/ur16e/ur16e_sim.ros2_control.xacro -->
<hardware>
  <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
  <param name="joint_commands_topic">/isaac_joint_commands</param>
  <param name="joint_states_topic">/isaac_joint_states</param>
</hardware>
<!-- Isaac이 position+velocity+effort를 모두 발행하므로 셋 다 선언 -->
<joint name="${tf_prefix}shoulder_pan_joint">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

### 5.5 카메라 프레임 — REP-103 optical frame
D405 매크로는 body frame(x-forward)과 광학 frame(z-forward/x-right/y-down)을 구분한다. 이 TF 정합이 모듈 7의 카메라 발행 `frameId`와 정확히 맞아야 sim/real 퍼셉션이 호환된다:

```xml
<!-- urdf/.../realsense_d405_macro.xacro -->
<joint name="${prefix}camera_depth_optical_joint" type="fixed">
  <!-- body(x fwd) -> optical(z fwd, x right, y down) -->
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
</joint>
<link name="${prefix}camera_depth_optical_frame"/>
```

> URDF의 `rpy=(-pi/2,0,-pi/2)`(body→optical)와, 모듈 7의 Isaac 카메라 코드 `_rpy(-pi/2,0,-pi/2)`가 **동일 변환**임을 대조하면 sim↔real TF 정합의 핵심이 한눈에 들어온다.

## 실습 (Hands-on)
1. `xacro`로 `ur16e_2f85_d405_sim.urdf.xacro`를 펼쳐(`xacro ... > out.urdf`) 링크/조인트 트리를 확인.
2. `collision:=false`로 바꿔 RViz에서 충돌 메쉬가 사라지는 것을 확인하고, MoveIt 충돌 계획과의 관계를 토론.
3. `robot_state_publisher`로 TF 트리를 띄워 `wrist_3_link → camera_*_optical_frame` 체인을 `ros2 run tf2_tools view_frames`로 시각화.

## 자주 만나는 함정
- 마스터 조인트 이름 불일치(`robotiq_85_left_knuckle_joint` vs `finger_joint`) → RViz가 그리퍼를 애니메이션 못 함(이 repo가 이름을 바꾼 이유).
- optical frame을 빼고 body frame에 이미지 발행 → 포인트클라우드가 90° 틀어짐.
- collision 메쉬와 SRDF 비활성 규칙 불일치 → START_STATE_IN_COLLISION.

## 정리 체크리스트
- [ ] URDF link/joint/visual/collision을 읽는다.
- [ ] mimic(URDF) ↔ PhysxMimicJointAPI(USD) 대응을 설명한다.
- [ ] body frame과 optical frame을 구분한다.

---

# 모듈 6. OmniGraph 기초 — 비주얼 프로그래밍과 실행 모델

## 학습 목표
- OmniGraph(Action Graph)의 Node/Attribute/Connection/Evaluator를 이해한다.
- execution(실행) 연결과 data(데이터) 연결의 차이를 안다.
- `og.Controller.edit`의 CREATE_NODES / CONNECT / SET_VALUES 3-키 패턴을 읽고 쓴다.
- `OnPlaybackTick`이 만드는 매 스텝 동기화를 설명한다.

## 강의 본문

### 6.1 OmniGraph란
OmniGraph는 Isaac Sim의 **노드 기반 비주얼 프로그래밍**이자 실행 엔진이다. 센서 발행, ROS 브리지, 로봇 구동 같은 "매 프레임 해야 하는 일"을 노드 그래프로 구성한다. GUI로도, 파이썬으로도 만들 수 있는데 repo는 **재현성을 위해 100% 파이썬(`og.Controller`)**으로 짠다.

핵심 용어:
- **Node**: 기능 단위(예: `ROS2PublishJointState`).
- **Attribute**: 노드의 입력/출력 핀. `inputs:execIn`, `outputs:tick`처럼 이름이 붙음.
- **Connection**: 핀과 핀을 잇는 선. 두 종류 —
  - **execution 연결**(`outputs:tick → inputs:execIn`): "언제 실행할지"의 흐름.
  - **data 연결**(`outputs:simulationTime → inputs:timeStamp`): "값"의 흐름.
- **Evaluator**: 그래프 평가 방식. 여기선 `"execution"`.

### 6.2 OnPlaybackTick — 동기화의 심장
`OnPlaybackTick`은 **렌더 스텝마다 tick을 1번** 쏜다. 이 tick을 여러 노드의 `execIn`에 부채살처럼 연결하면, 매 프레임 동일 시점에 그 노드들이 실행된다.

```python
# ur16e_isaac_ros2.py — ActionGraph
og.Controller.Keys.CONNECT: [
    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
    ...
]
```

### 6.3 `og.Controller.edit`의 3-키 패턴
repo의 모든 그래프가 동일 패턴이라, 이 구조만 익히면 전부 읽힌다:

```python
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [ (별칭, 노드타입), ... ],   # ① 노드 생성
        og.Controller.Keys.CONNECT:      [ (출력핀, 입력핀), ... ],   # ② 연결
        og.Controller.Keys.SET_VALUES:   [ (입력핀, 값), ... ],       # ③ 파라미터
    },
)
```

- **CREATE_NODES**: `("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState")` — 왼쪽은 내가 붙인 별칭, 오른쪽은 노드 타입 식별자.
- **CONNECT**: `"노드.outputs:핀" → "노드.inputs:핀"`.
- **SET_VALUES**: 토픽명, 대상 Prim 경로 등 상수 입력.

### 6.4 데이터 연결의 예 — 타임스탬프 공유
시뮬레이션 시간을 읽어 두 발행 노드에 같은 타임스탬프를 흘려준다(모듈 7의 `/clock` 동기화 기반):

```python
("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
```

## 실습 (Hands-on)
1. **GUI 먼저**: Isaac Sim의 Action Graph 에디터에서 `OnPlaybackTick` → `IsaacReadSimulationTime`를 손으로 연결해보고 핀 색(execution vs data)을 관찰.
2. **코드로 재현**: 같은 그래프를 최소 `og.Controller.edit`로 작성해 GUI 결과와 동일함을 확인.
3. ActionGraph의 CONNECT에서 `ArticulationController.inputs:execIn` 한 줄을 빼고 실행 → tick을 못 받아 로봇이 명령에 반응하지 않음을 관찰.

## 자주 만나는 함정
- execution 연결을 빠뜨림 → 노드가 "생성은 됐지만 한 번도 실행 안 됨"(가장 흔한 무반응 원인).
- 핀 이름 오타(`inputs:execIn` 등) → 조용히 연결 실패.
- 별칭과 노드 타입 식별자를 혼동.

## 정리 체크리스트
- [ ] execution vs data 연결을 구분한다.
- [ ] CREATE_NODES/CONNECT/SET_VALUES를 직접 쓴다.
- [ ] OnPlaybackTick의 역할을 설명한다.

---

# 모듈 7. OmniGraph + ROS2 Bridge — 로봇을 ROS2에 연결

## 학습 목표
- ROS2 Bridge 확장을 켜고 `ROS2Context`로 도메인을 잡는다.
- 관절 상태/명령/클록을 주고받는 제어 루프 그래프를 구성한다.
- 카메라(RGB/Depth/PointCloud/CameraInfo)를 ROS2로 발행한다.
- 제어/카메라/정적카메라를 **3개 그래프로 분리**하는 설계 이유를 안다.

## 강의 본문

### 7.1 브리지 확장과 컨텍스트
Isaac을 ROS2와 잇는 첫걸음은 확장 활성화다:

```python
# ur16e_isaac_ros2.py
extensions.enable_extension("isaacsim.ros2.bridge")
```

각 그래프는 `ROS2Context` 노드 하나를 만들어 그 그래프의 ROS2 노드들이 공유한다(도메인/DDS 수명 관리).

### 7.2 제어 루프 그래프(ActionGraph)
이 그래프가 "로봇이 움직이는" 핵심이다. 노드 구성:

```python
CREATE_NODES: [
    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
    ("ReadSimTime",   "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("Context",       "isaacsim.ros2.bridge.ROS2Context"),
    ("PublishJointState",   "isaacsim.ros2.bridge.ROS2PublishJointState"),   # 상태 발행
    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"), # 명령 수신
    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"), # 구동
    ("PublishClock",  "isaacsim.ros2.bridge.ROS2PublishClock"),              # 시간 발행
]
```

명령 흐름(ROS → Isaac): 구독 노드가 받은 명령을 ArticulationController에 데이터 연결로 흘린다.

```python
("SubscribeJointState.outputs:jointNames",       "ArticulationController.inputs:jointNames"),
("SubscribeJointState.outputs:positionCommand",  "ArticulationController.inputs:positionCommand"),
("SubscribeJointState.outputs:velocityCommand",  "ArticulationController.inputs:velocityCommand"),
("SubscribeJointState.outputs:effortCommand",    "ArticulationController.inputs:effortCommand"),
```

토픽/대상 Prim 설정(모듈 4의 `ARTICULATION_ROOT`가 여기서 쓰인다):

```python
SET_VALUES: [
    ("ArticulationController.inputs:robotPath", ARTICULATION_ROOT),     # 어느 로봇을 구동?
    ("PublishJointState.inputs:topicName",   "isaac_joint_states"),
    ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
    ("PublishJointState.inputs:targetPrim",  [usdrt.Sdf.Path(ARTICULATION_ROOT)]),
]
```

전체 토픽 계약:

| 방향 | 토픽 | 노드 |
|---|---|---|
| Isaac→ROS | `/isaac_joint_states` | PublishJointState |
| ROS→Isaac | `/isaac_joint_commands` | SubscribeJointState |
| Isaac→ROS | `/clock` | PublishClock |

이 이름이 모듈 5의 ros2_control 플러그인 파라미터(`joint_commands_topic`/`joint_states_topic`)와 **정확히 일치**해야 모듈 8의 제어 스택이 붙는다.

### 7.3 시간 동기화와 use_sim_time
`IsaacReadSimulationTime → PublishClock`이 `/clock`을 발행하고, ROS 측은 `use_sim_time:=true`로 이 시간을 따른다. 결정론적·재현 가능한 타이밍의 기반이다. (같은 `simulationTime`을 JointState 타임스탬프에도 공유 — 모듈 6.4)

### 7.4 카메라 발행(CameraGraph)
`--with-camera`일 때 별도 그래프를 만든다. 하나의 RenderProduct(카메라 1대)에서 RGB·Depth·PointCloud·CameraInfo를 뽑아 **실제 realsense2_camera와 같은 토픽명**으로 발행 → sim/real 퍼셉션 스택이 동일하게 구독:

```python
CREATE_NODES: [
    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
    ("RGB",   "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ("DepthPCL", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ("ColorInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ("DepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
]
SET_VALUES: [
    ("RGB.inputs:type", "rgb"),     ("RGB.inputs:topicName", "/camera/color/image_raw"),
    ("RGB.inputs:frameId", "camera_color_optical_frame"),       # ★ 모듈 5의 optical frame과 일치
    ("Depth.inputs:type", "depth"), ("Depth.inputs:topicName", "/camera/depth/image_rect_raw"),
    ("DepthPCL.inputs:type", "depth_pcl"), ("DepthPCL.inputs:topicName", "/camera/depth/color/points"),
    ...
]
```

- 실행 흐름이 제어 그래프와 다른 점: tick → **RenderProduct** → 각 Helper로 `execOut`과 `renderProductPath`를 함께 전달:

```python
("OnTick.outputs:tick", "RenderProduct.inputs:execIn"),
("RenderProduct.outputs:execOut", "RGB.inputs:execIn"),
("RenderProduct.outputs:renderProductPath", "RGB.inputs:renderProductPath"),
```

### 7.5 왜 그래프를 3개로 나누나
| 그래프 | 트리거 | 역할 |
|---|---|---|
| `/ActionGraph` | 항상 | 관절 상태/명령/클록 (로봇 제어) |
| `/CameraGraph` | `--with-camera` | eye-in-hand D405 — **grasp 인지** |
| `/StaticCamGraph` | `--with-static-cam` | 작업공간 고정 카메라 — **nvblox 장애물 ESDF** |

- 관심사 분리: 각 그래프가 자기 `OnPlaybackTick`/`ROS2Context`를 가져 독립적으로 켜고 끌 수 있다(옵션 플래그).
- 역할 분리 메시지(강의 핵심): eye-in-hand는 손끝/물체를 보고, static cam은 팔에 안 가려진 안정적 월드맵을 본다 → 모듈 9에서 cuMotion/nvblox로 이어짐.

### 7.6 그래프는 언제 도나
그래프는 "구성"일 뿐, 실제로는 물리 초기화 후 매 스텝 렌더링에서 OnPlaybackTick이 돌린다:

```python
simulation_context.initialize_physics()
simulation_context.play()
while simulation_app.is_running():
    simulation_context.step(render=True)   # 매 스텝 tick -> 그래프 평가
```

## 실습 (Hands-on)
1. `ur16e_isaac_ros2.py` 실행 후 `ros2 topic list`로 `/isaac_joint_states`, `/isaac_joint_commands`, `/clock` 확인. `ros2 topic echo /isaac_joint_states --once`로 조인트 이름 검증.
2. `--with-camera`로 재실행하고 `ros2 topic hz /camera/depth/color/points`, RViz에서 PointCloud2 시각화.
3. `--with-static-cam --obstacle`로 정적 카메라와 장애물 박스를 띄워, 두 카메라 토픽 네임스페이스(`/camera/*` vs `/static_cam/*`)가 분리됨을 확인(모듈 9 예고).

## 자주 만나는 함정
- `ROS_DOMAIN_ID`가 ROS 측과 달라 토픽이 안 보임(스크립트 헤더가 경고).
- Isaac 토픽명과 ros2_control 파라미터 토픽명 불일치 → 제어 스택이 안 붙음.
- 카메라 `frameId`와 URDF optical frame 불일치 → TF/포인트클라우드 어긋남.
- 그래프는 만들었는데 `play()`/스텝을 안 돌려 tick이 없어 무반응.

## 정리 체크리스트
- [ ] ActionGraph의 명령/상태/클록 경로를 그린다.
- [ ] 토픽명·frameId 계약이 sim↔real 호환의 열쇠임을 설명한다.
- [ ] 3-그래프 분리(제어/grasp/장애물)의 이유를 든다.

---

## 모듈 2→7 연결 한눈에 보기
USD로 **로봇을 만들고**(2~4) → URDF로 **두 세계를 정합**하고(5) → OmniGraph로 **실행 그래프를 짜고**(6) → ROS2 Bridge로 **로봇을 ROS에 연결**(7). 이 다음(모듈 8~10)에서 ros2_control·MoveIt2가 이 토픽들 위에 올라타 "계획→실행→퍼셉션→실로봇"으로 확장된다.
