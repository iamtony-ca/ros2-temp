로그가 딱 핵심을 알려줘요:

gym.spec.max_episode_steps = 1000 → 타임리밋 오버라이드 적용 OK

terminated=True, truncated=False 에서 292스텝에 조기 종료 → 시간 제한이 아닌 과제 성공/실패 조건으로 에피소드가 끝남 (TransferCube는 ‘성공 시 종료’가 기본) 


그래서 “더 많은 액션을 한 영상에 담기”를 원한다면, 조기 종료를 내부적으로 ‘리셋 후 계속’ 처리해서 여러 에피소드를 하나의 긴 세션으로 연결(concat) 하는 게 제일 간단하고 안전해요. Gym은 OrderEnforcing 래퍼 때문에 terminated=True 이후에는 reset()을 먼저 해야 다음 step()을 받을 수 있는데, 우리는 환경 클래스 내부에서 바로 reset() 해버리고 “끝나지 않은 것처럼” 계속 굴리면 됩니다. (README의 래퍼 체인 참고: TimeLimit<OrderEnforcing<...>>) 

아래 패치만 추가해 주세요.


---

1) env.py — 에피소드 자동 이어붙이기(콘캣) 옵션

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

이렇게 하면 성공/실패로 terminated=True가 떠도 즉시 리셋하고 계속 진행합니다.

세션의 총 길이는 max_total_steps(없으면 무제한) 또는 런타임 max_hz와 함께 결정돼요.

Gym의 terminated/truncated 의미는 공식 문서대로이며(성공/실패 vs 시간/외생 제한), 지금처럼 성공 조건으로 조기 종료되는 건 자연스러운 동작이에요. 우리는 “데모용 긴 비디오”를 위해서만 그것을 부드럽게 이어붙여 쓰는 거고요. 



---

2) main.py — 세션 길이(예: 60초)로 끊어 저장

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

이제 여러 개의 내부 에피소드가 자동으로 이어져서 total_seconds 만큼 돌아가고, VideoSaver가 한 개 mp4로 내보냅니다.

파일이 너무 커지면 VideoSaver(subsample=2)로 저장 fps를 25로 낮출 수 있어요. (길이는 그대로, 용량만 줄어듦)



---

3) 확인 포인트

로그는 이제 terminated=True가 떠도 세션은 끝나지 않고 계속 진행합니다. (중간중간 장면이 리셋되어 “텔레포트”처럼 보이는 건 정상)

“하나의 긴 에피소드로만” 만들고 싶다면, 환경 자체의 성공 종료 조건을 더 빡세게 만들거나 끄는 식으로 gym-aloha 쪽을 커스터마이즈해야 합니다. 기본 사양상 TransferCube는 성공 시 종료가 맞아요. 



---

4) 대안 (원하면)

작업 자체를 바꾸기: AlohaInsertion-v0 등 조기 성공이 덜 발생하는 과제를 쓰면 한 에피소드가 더 길 수 있어요. 그래도 결국 성공 시 종료됩니다. 

여러 에피소드를 파일 하나로 합치기: 지금 방식이 바로 그겁니다(콘캣).

정말 한 에피소드만 길게: gym-aloha 내부 성공 판정 로직을 수정해야 하는데, 레포 기본 규칙은 “최대 보상(성공 기준)에 도달하면 종료”입니다. 



---

이대로 돌리면, 지금처럼 terminated=True가 200~300스텝에서 떠도 자동으로 reset→계속 진행해서 5초를 훨씬 넘는 비디오가 나와요. 필요하면 total_seconds를 원하는 길이로 조정해 주세요!
