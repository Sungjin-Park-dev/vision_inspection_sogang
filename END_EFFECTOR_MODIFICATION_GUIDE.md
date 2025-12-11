# End-Effector 추가 시 수정 가이드

`scripts/2_generate_trajectory.py`에서 로봇의 end-effector를 추가한 경우 수정해야 할 부분들을 정리한 문서입니다.

## 목차
- [빠른 시작 가이드](#빠른-시작-가이드) ⚡ **대부분의 경우 이것만 보면 됩니다**
- [왜 수정이 필요한가?](#왜-수정이-필요한가)
- [최소 수정 사항 (단순 Z축 연장 End-Effector)](#최소-수정-사항-단순-z축-연장-end-effector)
- [상세 가이드](#상세-가이드)
  1. [Configuration 수정 (common/config.py)](#1-configuration-수정-commonconfigpy)
  2. [URDF 경로 수정](#2-urdf-경로-수정)
  3. [Tool Offset 설정](#3-tool-offset-설정)
  4. [EAIK 변환 매트릭스 수정](#4-eaik-변환-매트릭스-수정)
  5. [Forward Kinematics 수정](#5-forward-kinematics-수정)
  6. [충돌 체크 설정](#6-충돌-체크-설정)
  7. [Working Distance 조정](#7-working-distance-조정)

---

## 빠른 시작 가이드

### TL;DR: URDF/YML만 바꾸면 되나요?

**아니요!** URDF와 Robot Config(YML)을 변경하면:
- ✅ **IK 계산**: End-effector 자동 반영 (EAIK/CuRobo가 URDF 사용)
- ✅ **충돌 체크**: End-effector 자동 포함
- ❌ **FK 계산**: 수동 수정 필요 (코드가 하드코딩된 DH 파라미터 사용)

### 단순 End-Effector (Z축 연장, 예: 카메라 마운트)의 경우

**3단계만 수정하면 됩니다:**

#### ✅ 1단계: `common/config.py` 수정 (2줄)

```python
# Line 80: URDF 경로 변경
DEFAULT_URDF_PATH = "/curobo/src/curobo/content/assets/robot/ur_description/ur20_with_end_effector.urdf"

# 파일 끝에 추가: End-effector tool offset
END_EFFECTOR_TOOL_Z_OFFSET = 0.15  # End-effector 길이 (meters) - 실제 값으로 변경
```

#### ✅ 2단계: `scripts/2_generate_trajectory.py` 수정 (4곳)

<details>
<summary><b>Line 309</b> - save_trajectory_csv() 함수 내부</summary>

```python
# 수정 전
R, pos = fk_single(q[:6], tool_z=0.0)

# 수정 후
R, pos = fk_single(q[:6], tool_z=config.END_EFFECTOR_TOOL_Z_OFFSET)
```
</details>

<details>
<summary><b>Line 728</b> - build_clusters_from_ik() 함수 내부</summary>

```python
# 수정 전
R, p = fk_batch(q_array, tool_z)

# 함수 시작 부분에서 tool_z 기본값 확인 (Line 703)
def build_clusters_from_ik(
    viewpoints: List[Viewpoint],
    tool_z: float = 0.0  # ← 이 기본값을 config.END_EFFECTOR_TOOL_Z_OFFSET로 변경
) -> Tuple[List[Dict], np.ndarray]:
```
</details>

<details>
<summary><b>Line 779-781</b> - compute_motion_cost() 함수 내부</summary>

```python
# 수정 전
def compute_motion_cost(
    q_a: np.ndarray, R_a: np.ndarray,
    q_b: np.ndarray, R_b: np.ndarray,
    lam_rot: float, tool_z: float
) -> float:
    # ...
    R_mid, p_mid = fk_single(q_mid, tool_z)
    _, p_a = fk_single(q_a, tool_z)
    _, p_b = fk_single(q_b, tool_z)

# 수정 후: 파라미터는 그대로, main()에서 호출 시 tool_z 값 전달
# (Line 1395, 1398에서 처리)
```
</details>

<details>
<summary><b>Line 1389, 1395, 1398</b> - main() 함수 내부</summary>

```python
# 수정 전
clusters, target_coords = build_clusters_from_ik(viewpoints)
order = build_visit_order_robot_cost(clusters, nbrs, args.lambda_rot, 0.0)
picked, total_cost = choose_ik_given_order(clusters, order, args.lambda_rot, 0.0)

# 수정 후
tool_z = config.END_EFFECTOR_TOOL_Z_OFFSET
clusters, target_coords = build_clusters_from_ik(viewpoints, tool_z=tool_z)
order = build_visit_order_robot_cost(clusters, nbrs, args.lambda_rot, tool_z)
picked, total_cost = choose_ik_given_order(clusters, order, args.lambda_rot, tool_z)
```
</details>

#### ✅ 3단계: Robot Config와 URDF 준비

1. **URDF 파일**: End-effector 링크 추가
2. **Robot Config (YML)**: End-effector collision spheres 추가

#### ✅ 실행 방법

```bash
# End-effector 포함된 robot config 지정
python scripts/2_generate_trajectory.py \
    --object sample \
    --num_viewpoints 163 \
    --robot ur20_with_end_effector.yml  # ← End-effector config 사용
```

### 수정 완료 체크리스트

- [ ] `config.py`: `DEFAULT_URDF_PATH` 변경
- [ ] `config.py`: `END_EFFECTOR_TOOL_Z_OFFSET` 추가
- [ ] `2_generate_trajectory.py` Line 309: FK tool_z 수정
- [ ] `2_generate_trajectory.py` Line 703: 기본값 수정
- [ ] `2_generate_trajectory.py` Line 1389-1398: tool_z 파라미터 전달
- [ ] URDF 및 YML 파일 준비

**이것으로 끝입니다!** 복잡한 end-effector가 아니라면 추가 수정 불필요합니다.

---

## 왜 수정이 필요한가?

### IK vs FK: 왜 다르게 처리되나?

```
┌─────────────────────────────────────────────────────────────┐
│  IK 계산 (Inverse Kinematics)                               │
│  역할: 목표 위치/자세 → 관절 각도 계산                      │
│  사용: EAIK, CuRobo IK Solver                               │
│  입력: URDF 파일 (End-effector 포함)                        │
│  결과: ✅ End-effector 자동 반영                            │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  FK 계산 (Forward Kinematics)                               │
│  역할: 관절 각도 → End-effector 위치/자세 계산              │
│  사용: 코드 내부 fk_single(), fk_batch()                    │
│  입력: 하드코딩된 DH 파라미터 (Line 612-614)                │
│  결과: ⚠️ 수동 수정 필요 (URDF 독립적)                      │
└─────────────────────────────────────────────────────────────┘
```

### FK가 사용되는 곳

1. **Line 309** (`save_trajectory_csv`): CSV 파일에 저장할 target position/rotation 계산
2. **Line 728** (`build_clusters_from_ik`): Cluster 생성 시 각 IK solution의 위치 계산
3. **Line 779-781** (`compute_motion_cost`): GTSP 최적화 시 motion cost 계산

### 핵심 문제

```python
# Line 612-614: 하드코딩된 DH 파라미터 (UR20 6-DOF만 포함)
_DH_A = np.array([0.0, -0.612, -0.5723, 0.0, 0.0, 0.0], dtype=np.float64)
_DH_D = np.array([0.1807, 0.0, 0.0, 0.163941, 0.1157, 0.0922], dtype=np.float64)
_DH_ALPHA = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0], dtype=np.float64)
```

이 DH 파라미터는 **URDF 파일과 무관**하게 코드에 하드코딩되어 있습니다. URDF만 변경해도 FK 계산에는 반영되지 않습니다!

---

## 최소 수정 사항 (단순 Z축 연장 End-Effector)

대부분의 경우 (카메라 마운트, 단순 gripper 등), end-effector는 Z축으로만 연장됩니다. 이 경우 **tool_z 파라미터**만 수정하면 됩니다.

### 수정 위치 요약표

| 파일 | Line | 함수/위치 | 수정 내용 | 필수 여부 |
|------|------|----------|-----------|-----------|
| `config.py` | 80 | URDF 경로 | `DEFAULT_URDF_PATH` 변경 | ✅ 필수 |
| `config.py` | 끝 | 새 설정 | `END_EFFECTOR_TOOL_Z_OFFSET` 추가 | ✅ 필수 |
| `2_generate_trajectory.py` | 309 | `save_trajectory_csv` | `tool_z=0.0` → `tool_z=config.END_EFFECTOR_TOOL_Z_OFFSET` | ✅ 필수 |
| `2_generate_trajectory.py` | 703 | `build_clusters_from_ik` | 기본값 `tool_z=0.0` → `tool_z=config.END_EFFECTOR_TOOL_Z_OFFSET` | ✅ 필수 |
| `2_generate_trajectory.py` | 1389-1398 | `main` | tool_z 파라미터 명시적 전달 | ✅ 필수 |

### 상세 수정 내용

#### 1. `common/config.py` 수정

**Line 80 수정:**
```python
# 수정 전
DEFAULT_URDF_PATH = "/curobo/src/curobo/content/assets/robot/ur_description/ur20.urdf"

# 수정 후
DEFAULT_URDF_PATH = "/curobo/src/curobo/content/assets/robot/ur_description/ur20_with_end_effector.urdf"
```

**파일 끝에 추가 (Line 216 이후):**
```python
# ============================================================================
# End-Effector Configuration
# ============================================================================

# End-effector tool offset along Z-axis (meters)
# 카메라 또는 gripper가 tool flange에서 얼마나 떨어져 있는지
END_EFFECTOR_TOOL_Z_OFFSET = 0.15  # TODO: 실제 end-effector 길이로 변경
```

#### 2. `scripts/2_generate_trajectory.py` 수정

**Line 309 수정 (`save_trajectory_csv` 함수 내부):**
```python
# 수정 전 (Line 309)
R, pos = fk_single(q[:6], tool_z=0.0)

# 수정 후
R, pos = fk_single(q[:6], tool_z=config.END_EFFECTOR_TOOL_Z_OFFSET)
```

**Line 703 수정 (`build_clusters_from_ik` 함수 정의):**
```python
# 수정 전 (Line 701-704)
def build_clusters_from_ik(
    viewpoints: List[Viewpoint],
    tool_z: float = 0.0
) -> Tuple[List[Dict], np.ndarray]:

# 수정 후
def build_clusters_from_ik(
    viewpoints: List[Viewpoint],
    tool_z: float = config.END_EFFECTOR_TOOL_Z_OFFSET
) -> Tuple[List[Dict], np.ndarray]:
```

**Line 1386-1400 수정 (`main` 함수 내부):**
```python
# 수정 전 (Line 1386-1400)
# Step 5: Build GTSP
print("[5/7] Optimizing visit order (GTSP)...")

clusters, target_coords = build_clusters_from_ik(viewpoints)
print(f"  Built {len(clusters)} clusters")

nbrs = build_neighbors_knn(target_coords, args.knn)
print(f"  Built k-NN graph (k={args.knn})")

order = build_visit_order_robot_cost(clusters, nbrs, args.lambda_rot, 0.0)
print(f"  Visit order determined")

picked, total_cost = choose_ik_given_order(clusters, order, args.lambda_rot, 0.0)
print(f"  ✓ IK selection optimized (cost: {total_cost:.2f})")
print()

# 수정 후
# Step 5: Build GTSP
print("[5/7] Optimizing visit order (GTSP)...")

# End-effector tool offset
tool_z = config.END_EFFECTOR_TOOL_Z_OFFSET

clusters, target_coords = build_clusters_from_ik(viewpoints, tool_z=tool_z)
print(f"  Built {len(clusters)} clusters")

nbrs = build_neighbors_knn(target_coords, args.knn)
print(f"  Built k-NN graph (k={args.knn})")

order = build_visit_order_robot_cost(clusters, nbrs, args.lambda_rot, tool_z)
print(f"  Visit order determined")

picked, total_cost = choose_ik_given_order(clusters, order, args.lambda_rot, tool_z)
print(f"  ✓ IK selection optimized (cost: {total_cost:.2f})")
print()
```

### 테스트 방법

**1. FK 검증:**
```python
# Python 콘솔에서 테스트
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
from common import config
from scripts.generate_trajectory import fk_single

# Zero configuration에서 end-effector 위치 확인
q_test = np.zeros(6)
R, p = fk_single(q_test, tool_z=config.END_EFFECTOR_TOOL_Z_OFFSET)
print(f"End-effector position at zero config: {p}")
print(f"Expected Z offset: {config.END_EFFECTOR_TOOL_Z_OFFSET}")

# Z 값이 기본 UR20 + END_EFFECTOR_TOOL_Z_OFFSET인지 확인
```

**2. 전체 파이프라인 테스트:**
```bash
# Visualization으로 확인
python scripts/2_generate_trajectory.py \
    --object sample \
    --num_viewpoints 163 \
    --robot ur20_with_end_effector.yml \
    --visualize
```

---

## 상세 가이드

## 1. Configuration 수정 (common/config.py)

End-effector 관련 설정을 `common/config.py`에 추가해야 합니다.

```python
# End-effector configuration
END_EFFECTOR_LENGTH = 0.15  # End-effector 길이 (meters)
END_EFFECTOR_TOOL_Z_OFFSET = 0.15  # Tool frame Z-axis offset (meters)

# URDF path with end-effector
DEFAULT_URDF_PATH = "/path/to/ur20_with_end_effector.urdf"
```

**파일 위치**: `common/config.py`

---

## 2. URDF 경로 수정

### 수정 위치: Line 437
**함수**: `compute_ik_eaik()`

```python
# 수정 전
if urdf_path is None:
    urdf_path = config.DEFAULT_URDF_PATH

# 수정 후
if urdf_path is None:
    urdf_path = config.DEFAULT_URDF_PATH  # End-effector 포함된 URDF 사용
```

### 수정 위치: Line 1347
**함수**: `main()`

```python
# 로봇 설정 로드 시 end-effector 포함된 설정 사용
robot_cfg = load_yaml(join_path(get_robot_configs_path(), args.robot))["robot_cfg"]
```

**참고**: `args.robot`에 end-effector가 포함된 robot config 파일 경로를 전달해야 합니다.

---

## 3. Tool Offset 설정

End-effector의 길이만큼 tool offset을 설정해야 합니다.

### 수정 위치: Line 703, 728, 779, 780, 781
**함수**: `build_clusters_from_ik()`, `compute_motion_cost()`, `fk_single()`

```python
# 수정 전
tool_z: float = 0.0

# 수정 후
tool_z: float = config.END_EFFECTOR_TOOL_Z_OFFSET
```

### 예시: Line 728 (build_clusters_from_ik)
```python
# 수정 전
def build_clusters_from_ik(
    viewpoints: List[Viewpoint],
    tool_z: float = 0.0
) -> Tuple[List[Dict], np.ndarray]:

# 수정 후
def build_clusters_from_ik(
    viewpoints: List[Viewpoint],
    tool_z: float = config.END_EFFECTOR_TOOL_Z_OFFSET
) -> Tuple[List[Dict], np.ndarray]:
```

### 수정 위치: Line 1389, 1395, 1398
**함수**: `main()`에서 함수 호출 시

```python
# 수정 전
clusters, target_coords = build_clusters_from_ik(viewpoints)
order = build_visit_order_robot_cost(clusters, nbrs, args.lambda_rot, 0.0)
picked, total_cost = choose_ik_given_order(clusters, order, args.lambda_rot, 0.0)

# 수정 후
tool_z = config.END_EFFECTOR_TOOL_Z_OFFSET
clusters, target_coords = build_clusters_from_ik(viewpoints, tool_z=tool_z)
order = build_visit_order_robot_cost(clusters, nbrs, args.lambda_rot, tool_z)
picked, total_cost = choose_ik_given_order(clusters, order, args.lambda_rot, tool_z)
```

### 수정 위치: Line 309
**함수**: `save_trajectory_csv()` - FK 계산 시

```python
# 수정 전
R, pos = fk_single(q[:6], tool_z=0.0)

# 수정 후
R, pos = fk_single(q[:6], tool_z=config.END_EFFECTOR_TOOL_Z_OFFSET)
```

---

## 4. EAIK 변환 매트릭스 수정

End-effector의 좌표계가 기본 tool frame과 다른 경우, 변환 매트릭스를 수정해야 합니다.

### 수정 위치: Line 82-90
**상수**: `CUROBO_TO_EAIK_TOOL`

```python
# 수정 전 (기본 UR20 tool frame)
CUROBO_TO_EAIK_TOOL = np.array(
    [
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

# 수정 후 (End-effector 좌표계 반영)
# End-effector의 tool frame 변환에 맞게 조정
CUROBO_TO_EAIK_TOOL = np.array(
    [
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 1.0, 0.0, config.END_EFFECTOR_TOOL_Z_OFFSET],  # Z offset 추가
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
```

**참고**: End-effector의 좌표계 정의에 따라 회전 행렬도 조정이 필요할 수 있습니다.

---

## 5. Forward Kinematics 수정

End-effector를 포함한 FK 계산을 위해 DH 파라미터를 추가하거나 수정해야 합니다.

### 수정 위치: Line 612-614
**상수**: DH Parameters

```python
# 수정 전 (UR20 6-DOF)
_DH_A = np.array([0.0, -0.612, -0.5723, 0.0, 0.0, 0.0], dtype=np.float64)
_DH_D = np.array([0.1807, 0.0, 0.0, 0.163941, 0.1157, 0.0922], dtype=np.float64)
_DH_ALPHA = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0], dtype=np.float64)

# 수정 후 (End-effector 포함 - 예시)
# End-effector를 추가 링크로 포함하거나, tool_z로 처리
# Option 1: tool_z 파라미터로 처리 (현재 코드 유지)
# Option 2: DH 파라미터에 end-effector 추가
_DH_A = np.array([0.0, -0.612, -0.5723, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
_DH_D = np.array([0.1807, 0.0, 0.0, 0.163941, 0.1157, 0.0922, config.END_EFFECTOR_LENGTH], dtype=np.float64)
_DH_ALPHA = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0, 0.0], dtype=np.float64)
```

### 수정 위치: Line 617-650
**함수**: `fk_single()`

End-effector를 DH 파라미터로 추가하는 경우:

```python
def fk_single(q: np.ndarray, tool_z: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Forward kinematics for single configuration using DH parameters

    Args:
        q: Joint angles (6,) in radians
        tool_z: Additional tool offset along Z-axis (meters)

    Returns:
        R: 3x3 rotation matrix
        p: 3D position (meters)
    """
    T = np.eye(4, dtype=np.float64)

    # UR20 6 joints
    for i in range(6):
        th = q[i]
        ca, sa = np.cos(_DH_ALPHA[i]), np.sin(_DH_ALPHA[i])
        ct, st = np.cos(th), np.sin(th)

        A = np.array([
            [ct, -st*ca,  st*sa, _DH_A[i]*ct],
            [st,  ct*ca, -ct*sa, _DH_A[i]*st],
            [0.0, sa,     ca,    _DH_D[i]],
            [0.0, 0.0,    0.0,   1.0]
        ], dtype=np.float64)

        T = T @ A

    # End-effector transformation (fixed joint)
    # End-effector DH parameters (index 6)
    if len(_DH_A) > 6:
        ca, sa = np.cos(_DH_ALPHA[6]), np.sin(_DH_ALPHA[6])
        A_ee = np.array([
            [1.0, 0.0,   0.0,  _DH_A[6]],
            [0.0, ca,   -sa,   0.0],
            [0.0, sa,    ca,   _DH_D[6]],
            [0.0, 0.0,   0.0,  1.0]
        ], dtype=np.float64)
        T = T @ A_ee

    # Apply additional tool offset
    if tool_z != 0.0:
        T[:3, 3] += T[:3, 2] * tool_z

    return T[:3, :3], T[:3, 3]
```

**권장**: 간단한 경우 `tool_z` 파라미터로 처리하고, 복잡한 end-effector는 DH 파라미터 추가

---

## 6. 충돌 체크 설정

End-effector를 충돌 체크에 포함시켜야 합니다.

### Robot Config 파일 수정

End-effector 메시를 포함한 robot config 파일을 사용해야 합니다.

**파일 위치**: `curobo/robot_configs/ur20_with_end_effector.yml` (예시)

```yaml
robot_cfg:
  kinematics:
    collision_spheres:
      # UR20 기본 collision spheres
      # ...

      # End-effector collision spheres 추가
      end_effector_link:
        - center: [0.0, 0.0, 0.075]  # End-effector 중심
          radius: 0.03
        - center: [0.0, 0.0, 0.15]   # End-effector 끝
          radius: 0.02

    # End-effector 링크 정의
    link_names:
      - base_link
      - shoulder_link
      - upper_arm_link
      - forearm_link
      - wrist_1_link
      - wrist_2_link
      - wrist_3_link
      - end_effector_link  # 추가
```

### 수정 위치: Line 1347-1358
**함수**: `main()` - IK Solver 설정

```python
# Robot config에 end-effector 포함 확인
robot_cfg = load_yaml(join_path(get_robot_configs_path(), args.robot))["robot_cfg"]

# IK Solver가 end-effector를 포함한 collision spheres를 사용
ik_config = IKSolverConfig.load_from_robot_config(
    robot_cfg,
    world_cfg,
    self_collision_check=True,
    collision_checker_type=CollisionCheckerType.MESH,
    tensor_args=tensor_args,
    use_cuda_graph=True,
)
```

---

## 7. Working Distance 조정

End-effector 추가로 카메라 위치가 변경되므로, working distance를 조정해야 합니다.

### 수정 위치: Line 1288-1293
**함수**: `main()`

```python
# 수정 전
working_distance_m = config.CAMERA_WORKING_DISTANCE_MM / 1000.0
if 'camera_spec' in metadata and 'working_distance_mm' in metadata['camera_spec']:
    working_distance_m = metadata['camera_spec']['working_distance_mm'] / 1000.0

# 수정 후
# End-effector 길이를 고려한 working distance 계산
base_working_distance_m = config.CAMERA_WORKING_DISTANCE_MM / 1000.0
if 'camera_spec' in metadata and 'working_distance_mm' in metadata['camera_spec']:
    base_working_distance_m = metadata['camera_spec']['working_distance_mm'] / 1000.0

# End-effector 오프셋 반영
# 카메라가 end-effector 끝에 있는 경우
working_distance_m = base_working_distance_m  # End-effector 길이는 이미 FK에 반영됨
```

### Configuration 파일 수정

`common/config.py`에서:

```python
# 수정 전
CAMERA_WORKING_DISTANCE_MM = 400.0  # 기본 working distance

# 수정 후
# End-effector를 고려한 working distance
# End-effector 끝에서 물체까지의 거리
CAMERA_WORKING_DISTANCE_MM = 400.0  # End-effector 끝에서의 거리
# 또는
CAMERA_WORKING_DISTANCE_MM = 400.0 - (END_EFFECTOR_LENGTH * 1000.0)  # Tool flange에서의 거리
```

---

## 요약: 수정 체크리스트

### ✅ 필수 수정 사항 (단순 Z축 연장 End-Effector)

**대부분의 경우 아래 항목만 수정하면 됩니다:**

- [ ] `common/config.py`: End-effector 설정 추가
  - [ ] Line 80: `DEFAULT_URDF_PATH` 변경
  - [ ] 파일 끝: `END_EFFECTOR_TOOL_Z_OFFSET` 추가
- [ ] `scripts/2_generate_trajectory.py`: FK tool_z 수정
  - [ ] Line 309: `save_trajectory_csv()` 내부
  - [ ] Line 703: `build_clusters_from_ik()` 기본값
  - [ ] Line 1389-1398: `main()` - tool_z 파라미터 전달
- [ ] Robot config/URDF 파일
  - [ ] URDF: End-effector 링크 추가
  - [ ] YML: End-effector collision spheres 추가

**총 수정 파일: 2개 (config.py, 2_generate_trajectory.py)**

### ⚠️ 선택적 수정 사항 (복잡한 End-Effector)

**아래는 특수한 경우에만 필요합니다:**

- [ ] Line 82-90: `CUROBO_TO_EAIK_TOOL` 변환 매트릭스
  - 필요 조건: End-effector 좌표계가 UR20 tool frame과 다른 경우
  - 예시: 회전된 카메라, 특수 gripper
- [ ] Line 612-614: DH 파라미터 추가
  - 필요 조건: End-effector가 복잡한 구조 (여러 링크, 회전 조인트)
  - 예시: Multi-joint gripper
- [ ] Line 617-650: `fk_single()` 함수 로직 수정
  - 필요 조건: DH 파라미터를 수정한 경우
- [ ] Line 1288-1293: Working distance 조정
  - 필요 조건: 카메라 작업 거리 재계산이 필요한 경우
  - 대부분 불필요 (FK에서 이미 처리됨)

---

## 테스트 방법

1. **FK 검증**
   ```python
   # Test FK with end-effector
   q_test = np.zeros(6)
   R, p = fk_single(q_test, tool_z=config.END_EFFECTOR_TOOL_Z_OFFSET)
   print(f"End-effector position: {p}")
   # Expected: Z축으로 END_EFFECTOR_TOOL_Z_OFFSET만큼 추가된 위치
   ```

2. **IK 검증**
   ```python
   # Test IK with end-effector
   # Run script with --visualize flag
   python scripts/2_generate_trajectory.py --object sample --num_viewpoints 163 --visualize
   ```

3. **충돌 체크 검증**
   ```python
   # Check collision detection includes end-effector
   # Monitor collision check debug output
   ```

---

## 참고 자료

- URDF 파일: `ur20_description/ur20.urdf`
- Robot config: `curobo/robot_configs/`
- Common config: `common/config.py`
- CuRobo documentation: https://curobo.org/

---

## 문의 사항

End-effector 설정 관련 문의:
1. End-effector의 정확한 치수 확인
2. End-effector URDF 파일 확인
3. Tool frame 좌표계 정의 확인

---

## 빠른 참조 (Quick Reference)

### 핵심 개념

```
IK (EAIK/CuRobo) → URDF 사용 → End-effector 자동 반영 ✅
FK (코드 내부)   → DH 하드코딩 → tool_z 파라미터 수정 필요 ⚠️
```

### 최소 수정 (대부분의 경우)

| 파일 | 수정 내용 |
|------|----------|
| `config.py` | `DEFAULT_URDF_PATH` 변경 + `END_EFFECTOR_TOOL_Z_OFFSET` 추가 |
| `2_generate_trajectory.py` | Line 309, 703, 1389-1398 (tool_z 파라미터) |
| URDF/YML | End-effector 링크 + collision spheres |

### 실행 예시

```bash
python scripts/2_generate_trajectory.py \
    --object sample \
    --num_viewpoints 163 \
    --robot ur20_with_end_effector.yml
```

### 트러블슈팅

**Q: IK는 잘 되는데 trajectory가 이상해요**
→ FK tool_z 파라미터 확인 (Line 309, 703, 1389-1398)

**Q: 충돌이 제대로 감지되지 않아요**
→ Robot config의 collision spheres 확인

**Q: End-effector 위치가 정확하지 않아요**
→ `END_EFFECTOR_TOOL_Z_OFFSET` 값 확인, FK 테스트 실행

---

**마지막 업데이트**: 2025-12-11
