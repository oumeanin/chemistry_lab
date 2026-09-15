# chemistry_lab — Isaac Sim 화학 실험실 + 사고 대응 데모

메카넘 베이스 + SO-101 듀얼암 로봇이 있는 화학 실험실(Chemistry3D lab) 씬과,
**비커 낙하 → 용액 유출 → 유독가스 → 사이렌** 순서로 자동 재생되는 사고 상황 데모 레이어입니다.

| 파일 | 설명 |
|---|---|
| `chem_lab_scene.usda` | 기본 씬: 실험실 + 로봇 + ROS2 그래프 (팀 원본, 그대로 유지) |
| `chem_lab_demo.usda` | **사고 데모 씬** — 기본 씬을 서브레이어로 깔고 그 위에 효과를 얹음 |
| `build_demo_layer.py` | `chem_lab_demo.usda` 생성 스크립트 (수정은 여기서, usda는 직접 편집하지 않음) |
| `build_chem_lab_scene.py` | 기본 씬 생성 스크립트 (팀 원본) |

---

## 1. 받기

```bash
# git-lfs 필요 (USD/텍스처 150MB가 LFS)
sudo apt install git-lfs && git lfs install

git clone https://github.com/oumeanin/chemistry_lab.git
cd chemistry_lab
git lfs pull
```

`chem_lab/lab.usd`가 **22MB**면 정상, **133바이트**면 LFS 포인터만 받은 것 → `git lfs pull` 다시.
(GitHub "Download ZIP"으로 받으면 LFS 파일이 빠지므로 반드시 clone.)

## 2. 데모 열기

Isaac Sim **5.1 / 6.0.1** 모두 열립니다 (5.1 노트북 RTX 3070 8GB에서 제작·검증).

```bash
conda activate <isaacsim env>
isaacsim isaacsim.exp.full --enable omni.flowusd
```

→ **File > Open > `chem_lab_demo.usda`** → **Play (Space)**

* `--enable omni.flowusd` 가 없으면 **가스가 안 보입니다** (Flow 확장은 기본 비활성).
* 첫 Play는 SDF 콜라이더 쿠킹 + GPU 다이내믹스 초기화로 수십 초 걸릴 수 있음.
* 열리면 뷰포트가 실험실 내부(로봇 + 실험대)를 보는 시점으로 시작. 카메라 메뉴에서 `/Cameras/LabView`로 언제든 복귀.
* 데모 씬은 `/physicsScene`에 **GPU 다이내믹스**를 켭니다 (파티클 유체 필수). 기본 씬은 영향 없음.

## 3. 무슨 일이 일어나나 (Play 기준)

| 시각 | 사건 | 프림 |
|---|---|---|
| 0.0s | 위험 비커(초록 용액)가 실험대 앞 가장자리에서 통로 쪽으로 밀림 | `/DemoBeakers/HazardBeaker` |
| ~0.8s | 바닥 착지(옆으로 누움), 용액이 쏟아져 웅덩이 | `/DemoFX/HazardFluid` (PhysX 파티클 유체) |
| 1.0s | 웅덩이에서 초록 연기 | `/DemoFX/Gas/SpillFumes` |
| 2.0s | **사이렌**: 실험실 조명이 어두운 청색으로, 천장 스트로브 2개가 빨강 점멸 (1.2Hz) | `/DemoFX/Siren/*` |
| 2.0s | 벽 환기구 4곳 + 방 전체에서 가스 확산, ~3초면 방이 초록 연무로 가득 | `/DemoFX/Gas/Vent*`, `RoomFill` |
| ~10분 | 효과는 타임라인 끝까지 유지 (그 안에 로봇 접근/회수 촬영) | |

착지 위치: 로봇(`base_link`) 기준 **우측 약 1.1m, 앞 0.5m** 바닥, 대략 `(-5.07, 3.69, -2.75)`.
기본 씬에 있던 로봇 앞 빈 비커(`/Beaker`)는 데모에서 비활성화됨.

실험대에는 "적정 실험 중" 세팅: 뷰렛 스탠드, 색깔 용액이 든 비커 5개(하나는 전자저울 위), 삼각플라스크, 시약병, 분젠버너.
비커들은 모두 물리 강체라 로봇이 집을 수 있습니다.

## 4. 바꾸고 싶을 때

**usda를 직접 고치지 말고** `build_demo_layer.py` 상단 상수를 수정한 뒤 다시 생성합니다.

```bash
python build_demo_layer.py          # chem_lab_demo.usda 재생성 (Isaac Sim python으로 실행)
python build_demo_layer.py --test   # 생성 + 헤드리스 4초 시뮬: 비커 착지 위치, 유체 상태, 저울 위 비커 확인
```

| 하고 싶은 것 | 상수 |
|---|---|
| 사이렌 시작 시각 / 가스 시작 시각 | `T_SIREN`, `T_SPILL_GAS`, `T_ROOM_GAS` |
| 스트로브 세기 / 속도 / 켜짐 비율 | `BEACON_INTENSITY`, `BEACON_HZ`, `BEACON_DUTY` |
| 비상시 조명 밝기·색 | `LAB_LIGHTS` |
| 가스 색 / 방이 차는 속도 | `GAS_COLOR`, `make_gas()`의 `RoomFill` coupleRate (0.6) |
| 비커 배치 / 용액 색 | `DESK_BEAKERS`, `DESK_LIQUIDS`, `PROPS` |
| 위험 비커 위치 / 밀리는 속도 | `HAZARD_XY`, `HAZARD_VEL` |
| 용액 점성(슬라임 정도) | `make_fluid_material()`의 `viscosity`, `cohesion` |
| 효과 유지 시간 | `SetEndTimeCode(FPS * 600)` |

실험대 상판은 평평하지 않습니다(싱크, 턱). 비커를 옮길 땐 `--test`로 제자리에 서 있는지 확인하세요.
평평한 구간: 통로 쪽 스트립 `x ∈ [-5.26, -5.14]`, 앞 가장자리 띠 `y ∈ [3.92, 4.03]`, 싱크 오른쪽 `x ∈ [-4.78, -2.90]`.

## 5. 학교 서버(6.0.1, ROS2)에서 쓸 때

* `launch_isaac_ros.sh`로 띄우면 Flow가 안 켜지므로 `exec isaacsim ...` 줄에 `--enable omni.flowusd` 추가.
* `approach_beaker.py` / `grasp_beaker.py`는 `/Beaker`를 참조합니다. 데모 씬에서는
  `/DemoBeakers/HazardBeaker`로 바꾸고, 착지 위치는 고정값이 아니라 실제 위치를 읽어 쓰세요.
* ROS 그래프(`/FeedbackGraph`)는 기본 씬 것이 그대로 살아 있습니다.

## 6. 영상 촬영

노트북에서는 유체 + Flow + RTX를 실시간으로 돌리면 프레임이 떨어질 수 있습니다.
**Render > Movie Capture**로 오프라인 렌더하면 결과물은 매끄럽습니다. 카메라는 `/Cameras/LabView` 또는 새로 배치.
