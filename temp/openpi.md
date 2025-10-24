로그가 딱 핵심을 알려줘요:

* `gym.spec.max_episode_steps = 1000` → **타임리밋 오버라이드 적용 OK**
* `terminated=True, truncated=False` 에서 **292스텝에 조기 종료** → 시간 제한이 아닌 **과제 성공/실패 조건으로 에피소드가 끝남** (TransferCube는 ‘성공 시 종료’가 기본) ([GitHub][1])

그래서 “더 많은 액션을 한 영상에 담기”를 원한다면, **조기 종료를 내부적으로 ‘리셋 후 계속’ 처리**해서 여러 에피소드를 **하나의 긴 세션으로 연결(concat)** 하는 게 제일 간단하고 안전해요. Gym은 `OrderEnforcing` 래퍼 때문에 `terminated=True` 이후에는 `reset()`을 먼저 해야 다음 `step()`을 받을 수 있는데, 우리는 **환경 클래스 내부에서 바로 `reset()`** 해버리고 “끝나지 않은 것처럼” 계속 굴리면 됩니다. (README의 래퍼 체인 참고: `TimeLimit<OrderEnforcing<...>>`) ([GitHub][1])

아래 패치만 추가해 주세요.

---

## 1) `env.py` — 에피소드 자동 이어붙이기(콘캣) 옵션

```diff
 class AlohaSimEnvironment(_environment.Environment):
-    def __init__(self, task: str, obs_type: str = "pixels_agent_pos", seed: int = 0, max_episode_steps: int | None = None,) -> None:
+    def __init__(self,
+                 task: str,
+                 obs_type: str = "pixels_agent_pos",
+                 seed: int = 0,
+                 max_episode_steps: int | None = None,
+                 concat_episodes: bool = True,            # ★ 여러 에피소드를 한 세션으로 이어붙이기
+                 max_total_steps: int | None = None       # ★ 세션 전체 스텝 상한(없으면 무한)
+                 ) -> None:
         np.random.seed(seed)
         self._rng = np.random.default_rng(seed)
+        self._concat_episodes = concat_episodes
+        self._max_total_steps = max_total_steps
+        self._total_steps = 0

         make_kwargs = dict(obs_type=obs_type)
         if max_episode_steps is not None and max_episode_steps > 0:
             make_kwargs["max_episode_steps"] = max_episode_steps
         self._gym = gymnasium.make(task, **make_kwargs)

         self._last_obs = None
         self._done = True
         self._episode_reward = 0.0

     @override
     def reset(self) -> None:
         gym_obs, _ = self._gym.reset(seed=int(self._rng.integers(2**32 - 1)))
         self._last_obs = self._convert_observation(gym_obs)  # type: ignore
         self._done = False
         self._episode_reward = 0.0
+        # 콘캣 모드여도 reset()은 호출되지만, _total_steps는 유지 (세션 전체 카운트)

     @override
     def is_episode_complete(self) -> bool:
-        return self._done
+        # 세션(매크로 에피소드) 종료 기준
+        if self._max_total_steps is not None and self._total_steps >= self._max_total_steps:
+            return True
+        return self._done

     @override
     def get_observation(self) -> dict:
         if self._last_obs is None:
             raise RuntimeError("Observation is not set. Call reset() first.")
         return self._last_obs  # type: ignore

     @override
     def apply_action(self, action: dict) -> None:
-        gym_obs, reward, terminated, truncated, info = self._gym.step(action["actions"])
+        gym_obs, reward, terminated, truncated, info = self._gym.step(action["actions"])
+        self._total_steps += 1

         self._last_obs = self._convert_observation(gym_obs)  # type: ignore
-        self._done = terminated or truncated
+        if self._concat_episodes and (terminated or truncated):
+            # ★ 내부적으로 즉시 리셋해서 ‘계속 진행’
+            #   (OrderEnforcing 때문에 다음 step 전 reset이 필요)
+            gym_obs, _ = self._gym.reset(seed=int(self._rng.integers(2**32 - 1)))
+            self._last_obs = self._convert_observation(gym_obs)  # 다음 루프에서 바로 새 장면
+            self._done = False
+        else:
+            self._done = terminated or truncated

         self._episode_reward = max(self._episode_reward, reward)
```

* 이렇게 하면 **성공/실패로 `terminated=True`가 떠도 즉시 리셋하고 계속 진행**합니다.
* 세션의 총 길이는 `max_total_steps`(없으면 무제한) 또는 런타임 `max_hz`와 함께 결정돼요.
* Gym의 `terminated`/`truncated` 의미는 공식 문서대로이며(성공/실패 vs 시간/외생 제한), 지금처럼 성공 조건으로 조기 종료되는 건 자연스러운 동작이에요. 우리는 “데모용 긴 비디오”를 위해서만 그것을 **부드럽게 이어붙여** 쓰는 거고요. ([The Farama Foundation][2])

---

## 2) `main.py` — 세션 길이(예: 60초)로 끊어 저장

```diff
 @dataclasses.dataclass
 class Args:
     ...
     display: bool = False
     # 50Hz 기준 1000스텝이면 약 20초짜리 에피소드
     max_episode_steps: int | None = 1000
+    # 세션(여러 에피소드 이어붙인 총 길이) 목표: 60초 → 50Hz * 60 = 3000 스텝
+    total_seconds: int = 60
+    concat_episodes: bool = True

 def main(args: Args) -> None:
+    max_total_steps = 50 * args.total_seconds  # 런타임 max_hz=50 기준
     runtime = _runtime.Runtime(
         environment=_env.AlohaSimEnvironment(
             task=args.task,
             seed=args.seed,
-            max_episode_steps=args.max_episode_steps,
+            max_episode_steps=args.max_episode_steps,
+            concat_episodes=args.concat_episodes,
+            max_total_steps=max_total_steps,
         ),
         agent=_policy_agent.PolicyAgent(
             policy=action_chunk_broker.ActionChunkBroker(
                 policy=_websocket_client_policy.WebsocketClientPolicy(
                     host=args.host,
                     port=args.port,
                 ),
                 action_horizon=args.action_horizon,
             )
         ),
         subscribers=[
             _saver.VideoSaver(args.out_dir),
         ],
         max_hz=50,
     )
```

* 이제 **여러 개의 내부 에피소드가 자동으로 이어져서** `total_seconds` 만큼 돌아가고, `VideoSaver`가 **한 개 mp4**로 내보냅니다.
* 파일이 너무 커지면 `VideoSaver(subsample=2)`로 저장 fps를 25로 낮출 수 있어요. (길이는 그대로, 용량만 줄어듦)

---

## 3) 확인 포인트

* 로그는 이제 `terminated=True`가 떠도 **세션은 끝나지 않고 계속 진행**합니다. (중간중간 장면이 리셋되어 “텔레포트”처럼 보이는 건 정상)
* “하나의 긴 에피소드로만” 만들고 싶다면, **환경 자체의 성공 종료 조건을 더 빡세게 만들거나 끄는** 식으로 `gym-aloha` 쪽을 커스터마이즈해야 합니다. 기본 사양상 TransferCube는 성공 시 종료가 맞아요. ([GitHub][1])

---

## 4) 대안 (원하면)

* **작업 자체를 바꾸기:** `AlohaInsertion-v0` 등 조기 성공이 덜 발생하는 과제를 쓰면 한 에피소드가 더 길 수 있어요. 그래도 결국 성공 시 종료됩니다. ([GitHub][1])
* **여러 에피소드를 파일 하나로 합치기:** 지금 방식이 바로 그겁니다(콘캣).
* **정말 한 에피소드만 길게:** `gym-aloha` 내부 성공 판정 로직을 수정해야 하는데, 레포 기본 규칙은 “최대 보상(성공 기준)에 도달하면 종료”입니다. ([GitHub][1])

---

이대로 돌리면, 지금처럼 `terminated=True`가 200~300스텝에서 떠도 **자동으로 reset→계속 진행**해서 **5초를 훨씬 넘는 비디오**가 나와요. 필요하면 `total_seconds`를 원하는 길이로 조정해 주세요!

[1]: https://github.com/huggingface/gym-aloha "GitHub - huggingface/gym-aloha: A gym environment for ALOHA"
[2]: https://farama.org/Gymnasium-Terminated-Truncated-Step-API?utm_source=chatgpt.com "Deep Dive: Gymnasium Terminated - Truncated Step API"

############################  
#############################  

좋아, 두 가지를 한 번에 정리해줄게:

1. **`AlohaInsertion-v0`로 바꾸고 에피소드를 더 길게/여러 번 이어서 → 비디오 5초 넘기기**
2. **실시간으로 화면 보기(라이브 뷰)**

그리고 마지막에 네가 말한 “못 집었는데도 핸드오버(다른 팔에 건네기) 모션으로 넘어가는” 문제를 줄이는 팁도 같이 적었어.

---

# 1) `AlohaInsertion-v0` + 더 긴 녹화

지금 올려준 구조라면 다음 두 가지가 핵심이야.

* **타임리밋이 아니라 조기 종료(terminated=True)** 가 계속 뜨므로,
  **“여러 에피소드를 자동으로 이어붙이기(concat)”** 를 켜서 한 파일로 길게 찍는 게 현실적이야.
* 이전에 보낸 `concat_episodes / max_total_steps` 패치를 `env.py`에 넣으면, **성공/실패로 한 에피소드가 끝나도 즉시 reset → 계속 진행**하므로, **비디오 하나가 원하는 길이(예: 60초, 120초…)까지 쭉 이어져**.

이미 위 패치를 적용했다면 `main.py`에서 예를 들어 이렇게 실행하면 돼:

```python
# examples/aloha_sim/main.py (핵심 인자만)
@dataclasses.dataclass
class Args:
    task: str = "gym_aloha/AlohaInsertion-v0"
    max_episode_steps: int | None = 1000   # 개별 episode 한도(성공 전에 타임리밋 걸릴 일은 드묾)
    total_seconds: int = 60                # 전체 세션 길이(예: 60s)
    concat_episodes: bool = True           # 에피소드 붙이기
    action_horizon: int = 2                # ★ 반응성↑ (아래 3) 참조)

def main(args: Args) -> None:
    max_total_steps = 50 * args.total_seconds  # max_hz=50 기준
    runtime = _runtime.Runtime(
        environment=_env.AlohaSimEnvironment(
            task=args.task,
            seed=args.seed,
            max_episode_steps=args.max_episode_steps,
            concat_episodes=args.concat_episodes,
            max_total_steps=max_total_steps,
        ),
        ...
        subscribers=[
            _saver.VideoSaver(args.out_dir),
            # (2)에서 라이브뷰어도 여기에 추가
        ],
        max_hz=50,
    )
```

> 이렇게 하면 **조기 종료가 떠도 자동 reset**이라, **mp4 한 개가 60초(혹은 네가 준 길이)** 로 나와.

---

# 2) 실시간 보기(라이브)

옵션을 두 가지로 줄게. A(간단/빠른): **OpenCV 윈도우**. B(원격/헤드리스): **브라우저로 MJPEG 스트리밍**.

## A) OpenCV 윈도우로 바로 보기 (로컬 X11/Wayland)

### (A-1) `live_viewer.py` 구독자 추가

`examples/aloha_sim/live_viewer.py`:

```python
import numpy as np
import cv2
from openpi_client.runtime import subscriber as _subscriber
from typing_extensions import override

class LiveViewer(_subscriber.Subscriber):
    def __init__(self, window_name: str = "AlohaSim Live", every_n: int = 1):
        self._win = window_name
        self._n = max(1, every_n)
        self._i = 0
        cv2.namedWindow(self._win, cv2.WINDOW_NORMAL)

    @override
    def on_episode_start(self) -> None:
        pass

    @override
    def on_step(self, observation: dict, action: dict) -> None:
        self._i += 1
        if self._i % self._n:
            return
        im = observation["images"]["cam_high"]  # [C,H,W]
        im = np.transpose(im, (1, 2, 0))        # [H,W,C]
        im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        cv2.imshow(self._win, im)
        cv2.waitKey(1)

    @override
    def on_episode_end(self) -> None:
        cv2.waitKey(1)
```

### (A-2) `main.py`에 구독자 추가

```python
import live_viewer as _live

subscribers = [
    _saver.VideoSaver(args.out_dir),
    _live.LiveViewer(every_n=1),  # every_n=2로 가볍게도 가능
]
```

### (A-3) 도커에서 X11 연결 (Linux 호스트)

`compose.yml`의 `runtime` 서비스에 다음 추가:

```yaml
environment:
  - DISPLAY=${DISPLAY}
  - QT_X11_NO_MITSHM=1
  - MUJOCO_GL=glfw        # 창 띄우려면 egl 대신 glfw 권장
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
devices:
  - /dev/dri:/dev/dri
```

호스트에서:

```bash
xhost +local:
```

그리고 **OpenCV GUI용 라이브러리**가 필요해. Dockerfile에 다음 패키지 정도 추가 권장:

```dockerfile
RUN apt-get update && apt-get install -y \
    libglib2.0-0 libsm6 libxrender1 libxext6 \
    libgl1-mesa-glx libglfw3
# requirements.txt에 opencv-python 추가
```

> Wayland 환경이면 XWayland가 켜져 있어야 해. 안되면 `xhost`/`DISPLAY` 설정을 점검.

## B) 브라우저 스트리밍(MJPEG) – 헤드리스 서버에서도 OK

OpenCV 없이 **HTTP로 MJPEG 스트림**을 내보내면, 브라우저에서 `http://localhost:8090/stream`로 볼 수 있어.

`examples/aloha_sim/mjpeg_streamer.py`:

```python
import io, threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing_extensions import override
import imageio.v3 as iio
import numpy as np
from openpi_client.runtime import subscriber as _subscriber

class _FrameStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._jpg = None
    def update(self, rgb):
        # rgb: [H,W,C] uint8
        with self._lock:
            self._jpg = iio.imwrite("<bytes>", rgb, extension=".jpg")
    def get(self):
        with self._lock:
            return self._jpg

class _Handler(BaseHTTPRequestHandler):
    store: _FrameStore = None  # injected
    def do_GET(self):
        if self.path != "/stream":
            self.send_response(404); self.end_headers(); return
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                jpg = self.store.get()
                if jpg is None:
                    time.sleep(0.01); continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(1/20)  # ~20 FPS
        except BrokenPipeError:
            pass

class MjpegStreamer(_subscriber.Subscriber):
    def __init__(self, port: int = 8090, every_n: int = 1):
        self._store = _FrameStore()
        self._n = max(1, every_n); self._i = 0
        self._server = HTTPServer(("0.0.0.0", port), _Handler)
        self._server.RequestHandlerClass.store = self._store
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        print(f"[MJPEG] http://0.0.0.0:{port}/stream")

    @override
    def on_step(self, observation: dict, action: dict) -> None:
        self._i += 1
        if self._i % self._n: return
        im = observation["images"]["cam_high"]  # [C,H,W]
        im = np.transpose(im, (1,2,0))         # [H,W,C], RGB
        self._store.update(im)
```

`main.py`에:

```python
import mjpeg_streamer as _mjpeg
subscribers = [
    _saver.VideoSaver(args.out_dir),
    _mjpeg.MjpegStreamer(port=8090, every_n=1),
]
```

> 이 방식은 X11 없이도 되고, **브라우저로 바로 시청** 가능. (도커 포트는 host 네트워크라 그대로 접근)

---

# 3) “못 집었는데 핸드오버 모션으로 넘어가는” 문제 줄이기

이건 **정책(모델) 쪽의 오픈루프 습관** 때문에 생깁니다. 시퀀스 안에 “집기→들기→건네주기”가 들어있어 **성공 여부와 무관하게 다음 단계로 넘어가는 경향**이 있어요. 클라이언트에서 할 수 있는 현실적인 완화책:

* **`action_horizon` 값 낮추기 (권장)**
  지금 10이면 50 Hz 기준 0.2초 동안 동일 chunk를 유지합니다.
  **`action_horizon=1~2`** 로 줄이면 **매 스텝(또는 거의 매 스텝) 재계획**해서 관측에 더 민감하게 반응합니다.

* **여러 에피소드 붙여서 “계속 재시도”**
  위의 `concat_episodes=True` + 큰 `total_seconds`를 쓰면, **성공/실패로 종료돼도 즉시 reset** 되어 **재시도**가 이어집니다. “한 번 실패하면 바로 핸드오버로 넘어간다”는 느낌이 줄어요.

* **정책 재프롬프팅(가능한 경우)**
  만약 서버 정책이 프롬프트/지령을 받도록 되어 있다면(환경변수 `SERVER_ARGS` 옵션 제공 시),
  “*큐브를 확실히 쥐기 전에는 핸드오버/리프트 단계로 진행하지 말라*”, “*그립 실패 시 그립 포즈를 재탐색하고 각도/진입을 조정하면서 재시도하라*” 같은 문구를 **명시**하면 반응이 좋아집니다. (서버 옵션 노출이 없다면 생략)

* **관측 기반 가드(고급)**
  관측에서 **그립 성공 신호(예: 물체 높이 > 임계값, gripper dist < 임계값)** 가 없으면,
  **핸드오버에 해당하는 DOF를 억제**하는 에이전트 래퍼를 끼울 수 있습니다.
  (예: 기존 `_policy_agent.PolicyAgent`를 감싸서 `decide_action()` 결과를 후처리)
  다만 **액션 포맷(조인트/EE/그리퍼 index)** 을 정확히 알아야 하고, 정책 본래 동작을 바꾸는 거라 실험적인 방법이에요.

---

## 추천 세팅 요약

1. `task="gym_aloha/AlohaInsertion-v0"`
2. `action_horizon=2` (혹은 1)
3. `concat_episodes=True`, `total_seconds=60` (원하는 길이)
4. 실시간 보기:

   * 로컬이라면 **OpenCV LiveViewer + X11( MUJOCO_GL=glfw )**
   * 원격/헤드리스면 **MJPEG 스트리머**로 브라우저 시청

이 조합이면, **비디오는 5초를 훨씬 넘어서** 저장되고, **실시간 화면도 확인**할 수 있어.
그리고 “핸드오버로 성급히 넘어감” 문제도 **horizon↓ + concat 재시도**로 체감이 훨씬 나아질 거야.
############################  
############################  
좋은 포인트야. 결론부터:

* `SERVER_ARGS`는 **서버(모델) 띄울 때의 인자**를 넣는 곳이야. 예: `--env ALOHA_SIM` 또는 `policy:checkpoint --policy.config=... --policy.dir=...` 같은 것. **프롬프트(지령)는 여기가 아니라 “클라이언트가 매 스텝 보낼 observation 안에 넣는 것”**이 맞아. 공식 문서도 Remote Inference에서 서버는 `--env`로 체크포인트만 고르고, 실제 호출은 `{"...observation...", "prompt": task_instruction}` 형태로 보내라고 안내해. ([GitHub][1])

즉, “정책 재프롬프팅”을 하려면 **클라이언트 쪽에서 `prompt` 필드를 매번 같이 보내야** 하고, `SERVER_ARGS`로는 못 해.

---

## 어떻게 넣나? (너의 ALOHA_SIM 예제 기준)

지금 네 관측 딕셔너리는 대략 이렇게 생겼지:

```python
return {
    "state": gym_obs["agent_pos"],
    "images": {"cam_high": img},
}
```

여기에 원하는 지령을 **그대로** 추가해:

```python
return {
    "state": gym_obs["agent_pos"],
    "images": {"cam_high": img},
    "prompt": (
        "Insertion task. First ensure a reliable grasp before any handover. "
        "If grasp fails, reattempt grasp with adjusted approach; "
        "do not proceed to handover until the object is securely grasped."
    ),
}
```

> 서버는 observation 딕셔너리의 `prompt` 키를 해석해 액션을 생성하도록 되어 있어. 문서 예시도 동일하게 `prompt`를 같이 보내라고 하거든. ([GitHub][1])

### 더 깔끔하게(런타임에서 바꾸고 싶다면)

`main.py`에 인자로 받아서 전달하도록:

```python
@dataclasses.dataclass
class Args:
    ...
    prompt: str = ("Insertion task. Ensure a secure grasp before handover. "
                   "If grasp fails, retry grasp; do not hand over until secured.")

# PolicyAgent 앞단에서 observation에 prompt를 주입하는 간단 래퍼
class PromptInjectorPolicy:
    def __init__(self, base_policy, prompt: str):
        self._base = base_policy
        self._prompt = prompt
    def infer(self, obs: dict) -> dict:
        obs = dict(obs)
        obs["prompt"] = self._prompt
        return self._base.infer(obs)

# 기존 ActionChunkBroker에 래핑
policy=_websocket_client_policy.WebsocketClientPolicy(host=args.host, port=args.port)
policy = PromptInjectorPolicy(policy, args.prompt)
```

(또는 `env._convert_observation()`에 바로 `prompt`를 추가해도 되고, 위처럼 정책 앞단에서 주입해도 돼.)

---

## `SERVER_ARGS`는 어디에 쓰나?

* **프리셋 체크포인트로 서버 띄우기** (간단 모드):

  ```bash
  export SERVER_ARGS="--env ALOHA_SIM"
  docker compose -f examples/aloha_sim/compose.yml up --build
  ```

  → 내부적으로 `scripts/serve_policy.py --env=ALOHA_SIM`를 실행해 ALOHA용 체크포인트를 올려. ([GitHub][1])

* **원하는 체크포인트 지정해서 띄우기** (직접 모드):

  ```bash
  export SERVER_ARGS="policy:checkpoint --policy.config=pi05_libero --policy.dir=checkpoints/pi05_libero/your_run/20000"
  docker compose -f examples/aloha_sim/compose.yml up --build
  ```

  → 문서에 나온 그대로 `serve_policy.py policy:checkpoint ...` 형태. ([GitHub][2])

> 다시 강조하지만, **프롬프트는 여기서 못 바꿔**. 프롬프트는 **클라이언트가 observation에 `prompt`로** 넣어야 함. ([GitHub][1])

---

## “못 집었는데도 핸드오버로 넘어가는” 걸 줄이고 싶다면

1. 위처럼 **명시적 프롬프트**로 “안 잡히면 계속 재시도, 잡히기 전엔 핸드오버 금지”를 지시. (지금 이게 핵심)
2. `action_horizon`을 **1~2**로 낮춰 반응성을 올려(오픈루프 길이를 줄임).
3. 필요하면 내가 전에 준 **에피소드 콘캣**으로 실패-리셋-재시도 루프가 자연스럽게 이어지게.
4. 정말 강하게 제약하고 싶으면, **관측 신호 기반 후처리(가드)** 로 “그립 성공 전에 핸드오버 관련 DOF 억제” 같은 래퍼를 추가.

---

## 체크용 To-do

* 모델 서버 뜨는 로그 OK (`SERVER_ARGS`로 체크포인트 선택) ✔️
* 클라이언트에서 `prompt`가 실제 전송되는지 프린트로 확인 ✔️
* `action_horizon` 1~2로 테스트 ✔️
* 여전히 이상하면, 프롬프트 문구를 더 구체화(예: “**gripper width < X** and **object height > Y**일 때만 handover 단계 진행”)하고, 필요시 MJPEG/GUI 라이브뷰어로 실시간 확인 ✔️

필요하면 네 환경에 맞는 `SERVER_ARGS` 예시(ALOHA_SIM, DROID 등)랑, 프롬프트 문구 템플릿 몇 개 더 만들어줄게!

[1]: https://raw.githubusercontent.com/Physical-Intelligence/openpi/main/docs/remote_inference.md "raw.githubusercontent.com"
[2]: https://github.com/Physical-Intelligence/openpi "GitHub - Physical-Intelligence/openpi"

############################  
############################  
좋아, 두 가지를 바로 해결하자.

---

# 1) prompt가 “정말” 전송되는지 확인하는 가장 쉬운 방법 3가지

## (A) 클라이언트 쪽에서 바로 출력 (권장, 1분 컷)

`PromptInjectorPolicy` 래퍼에 **보내기 직전의 obs["prompt"]** 를 찍자.

```python
# examples/aloha_sim/main.py
class PromptInjectorPolicy:
    def __init__(self, base_policy, prompt: str, echo_every_n: int = 50):
        self._base = base_policy
        self._prompt = prompt
        self._i = 0
    def infer(self, obs: dict) -> dict:
        obs = dict(obs)
        obs["prompt"] = self._prompt
        self._i += 1
        if self._i % 50 == 1:  # 너무 시끄럽지 않게 N스텝마다 1번만
            print("[CLIENT] prompt ->", (obs["prompt"][:120] + "...") if len(obs["prompt"]) > 120 else obs["prompt"])
        return self._base.infer(obs)
```

적용:

```python
policy = _websocket_client_policy.WebsocketClientPolicy(host=args.host, port=args.port)
policy = PromptInjectorPolicy(policy, args.prompt, echo_every_n=50)
agent = _policy_agent.PolicyAgent(policy=action_chunk_broker.ActionChunkBroker(policy=policy, action_horizon=args.action_horizon))
```

→ 이건 “**보내기 직전** 내가 obs에 prompt를 넣었다”는 증명.

## (B) Subscriber로 관측에서 직접 확인

전송 경로 중간에서 **Runtime가 본 observation**을 찍어보는 방법.

```python
# examples/aloha_sim/prompt_debugger.py
from openpi_client.runtime import subscriber as _subscriber
from typing_extensions import override
class PromptDebugger(_subscriber.Subscriber):
    def __init__(self, every_n: int = 50): self._n=every_n; self._i=0
    @override
    def on_step(self, observation: dict, action: dict) -> None:
        self._i+=1
        if self._i % self._n==1 and "prompt" in observation:
            p = observation["prompt"]
            print("[RUNTIME] prompt in obs:", (p[:120]+"...") if isinstance(p,str) and len(p)>120 else p)
```

`main.py`:

```python
import prompt_debugger as _pdbg
subscribers = [
    _saver.VideoSaver(args.out_dir),
    _pdbg.PromptDebugger(every_n=50),
]
```

→ **환경→에이전트로 넘어가는 관측**에 prompt가 실려 있음을 확인.

## (C) 서버(모델)에서 수신 확인 (가능하면)

`openpi_server`의 WebSocket 핸들러(예: `serve_policy.py`의 `on_message`)에서 JSON 디코드 직후:

```python
print("[SERVER] got prompt:", message.get("prompt", None))
```

→ **서버가 실제로 받은 prompt**를 확인. (접근 가능할 때만)

> 추가 팁: 프롬프트에 타임스탬프 토큰(예: `PROMPT_ID=169...`)을 넣어두면 클/서 양쪽 로그에서 같은 토큰이 보이는지 매칭이 쉬워요.

---

# 2) action_horizon 낮추니 움직임만 빨라지고, “재시도 때 떨림” 발생 — 안정화 팩

문제 원인: horizon을 1~2로 낮추면 **고주파 제어**가 됩니다. 관측에 민감해져서 좋은 면도 있지만, 잡아야 할 때 미세 오차에 반응해 **액션이 좌우로 오락가락(챠터링)** → “제자리 떨림”.
해법은 3가지 층위에서 같이 쓰면 효과가 큽니다.

## (1) 저역통과(EMA) + 변화율 제한(rate limit) + 데드밴드

정책 출력을 “부드럽게” 만들고, 너무 작은 변화를 무시.

```python
import numpy as np

class FilteredPolicy:
    def __init__(self, base_policy, alpha: float = 0.5,  # EMA 0.0(강한 필터) ~ 1.0(무필터)
                 max_step_delta: float = 0.05,          # 1스텝 최대 변화량 제한
                 deadband: float = 1e-3):               # 아주 작은 변화는 0 취급
        self._base = base_policy
        self._alpha = alpha
        self._max_step_delta = max_step_delta
        self._deadband = deadband
        self._last = None

    def infer(self, obs: dict) -> dict:
        act = self._base.infer(obs)
        a = np.array(act["actions"], dtype=np.float32)

        if self._last is None:
            self._last = a.copy()
        # rate limit
        delta = np.clip(a - self._last, -self._max_step_delta, self._max_step_delta)
        a_limited = self._last + delta
        # EMA smoothing
        a_smooth = self._alpha * a_limited + (1 - self._alpha) * self._last
        # deadband
        a_smooth[np.abs(a_smooth - self._last) < self._deadband] = self._last[np.abs(a_smooth - self._last) < self._deadband]

        self._last = a_smooth
        act["actions"] = a_smooth.tolist()
        return act
```

적용 순서(프롬프트→필터→브로커):

```python
policy = _websocket_client_policy.WebsocketClientPolicy(host=args.host, port=args.port)
policy = PromptInjectorPolicy(policy, args.prompt, echo_every_n=50)
policy = FilteredPolicy(policy, alpha=0.6, max_step_delta=0.03, deadband=1e-3)
agent = _policy_agent.PolicyAgent(policy=action_chunk_broker.ActionChunkBroker(policy=policy, action_horizon=args.action_horizon))
```

> 파라미터 추천 시작값: `alpha=0.6`, `max_step_delta=0.03` (조인트 스케일에 맞춰 조정).

## (2) “급격한 상태전환 억제” — 최소 유지 스텝(min-hold)로 떨림 차단

특히 그리퍼/핸드오버처럼 **단계가 바뀌는 DOF**는 한 번 바뀌면 **N스텝 유지**:

```python
class MinHoldPolicy:
    def __init__(self, base_policy, hold_steps: int = 5, important_idx: list[int] = None):
        self._base = base_policy
        self._hold = hold_steps
        self._idx = important_idx or []  # 그리퍼/특정 축 인덱스
        self._last = None
        self._counter = 0

    def infer(self, obs: dict) -> dict:
        act = self._base.infer(obs)
        a = np.array(act["actions"], dtype=np.float32)
        if self._last is None:
            self._last = a.copy()
        # 중요한 축이 크게 바뀌면 그 상태를 hold_steps 동안 유지
        if any(abs(a[i] - self._last[i]) > 0.1 for i in self._idx):
            self._counter = self._hold
        if self._counter > 0:
            a[self._idx] = self._last[self._idx]  # 급격한 토글 억제(혹은 마지막 명령 유지)
            self._counter -= 1
        self._last = a
        act["actions"] = a.tolist()
        return act
```

적용 예:

```python
policy = MinHoldPolicy(policy, hold_steps=5, important_idx=[gripper_index])
```

> 인덱스는 해당 액션 포맷에 맞게 설정 필요.

## (3) 제어 주기 ↓ (max_hz를 50→30)

고주파 챠터링이 심하면 **샘플링 주기를 낮춰** 물리 안정성↑

```python
runtime = _runtime.Runtime(..., max_hz=30)  # 30Hz
```

---

# 3) “첫 실패 후 재시도 때 제대로 못 하고 떨림”을 더 줄이는 추가 팁

* **action_horizon=2~3**: 1은 최단 반응이지만 불안정해지기 쉬워요. 2~3으로 약간만 늘리면 안정-반응 절충 가능.
* **프롬프트를 더 구체화**:
  “*If the object is not grasped (gripper width > w_min AND object height < h_min), keep reattempting grasp with small pose adjustments. Do **not** proceed to handover or lifting.*” 처럼 **조건을 수식 수준으로 명확히**.
* **가드(후처리 제약)**: 관측으로 그립 실패가 확실하면, 핸드오버/리프트 관련 DOF를 0으로 **프로젝션**(강제 억제).
* **Env 관점**: 관측 잡음이 크면 카메라/픽셀 다운샘플/리사이즈/정규화/패딩 과정에서 미세 플리커가 생길 수 있음. `live_viewer`로 프레임을 눈으로 확인하고, 필요 시 `every_n=2`로 절반만 정책에 주는 것도 한 방법(지연↑ vs 안정성↑ trade-off).

---

## 빠른 체크리스트

1. (A) 또는 (B) 방식으로 **prompt 로그**가 실제로 찍히는지 확인
2. `action_horizon=2`로 시작
3. `FilteredPolicy(alpha=0.6, max_step_delta=0.03)` 적용
4. 필요 시 `MinHoldPolicy(hold_steps=5, important_idx=[gripper_idx])` 추가
5. 여전히 떨리면 `max_hz=30`로 낮춰 테스트

원하면, 네 액션 벡터 포맷(조인트/EE/그리퍼 인덱스)을 알려줘. 그에 맞춰 **지금 쓸 수 있는 인덱스/권장 파라미터**를 꽂아준 래퍼 코드를 바로 만들어줄게.
