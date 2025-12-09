# 비전 검사 파이프라인

로봇 비전 검사를 위한 3단계 파이프라인 구현

## 파이프라인 개요

```
1단계: 뷰포인트 생성
  입력:  멀티 머티리얼 메시 (OBJ + MTL)
  출력:  표면 뷰포인트 (위치 + 법선)
           ↓
2단계: 궤적 계획
  - IK 계산 (EAIK 해석적 솔버)
  - 방문 순서 최적화 (GTSP + 동적 프로그래밍)
  - 충돌 검사 및 재계획 (MotionGen)
  출력:  충돌 없는 관절 궤적
           ↓
3단계: 시뮬레이션
  - Isaac Sim에서 궤적 실행
  - 충돌 구체 시각화 (옵션)
```

## 사용법

### 필수 조건
- Isaac Sim + CuRobo 설치
- 입력 메시: `data/{object}/mesh/source.obj` (OBJ + MTL)

### 전체 실행 예시

```bash
# 1단계: 뷰포인트 생성 (개수 자동 계산)
/isaac-sim/python.sh scripts/1_create_viewpoint.py \
    --object sample \
    --material-rgb "170,163,158"
# 출력: data/sample/viewpoint/163/viewpoints.h5

# 2단계: 궤적 생성
/isaac-sim/python.sh scripts/2_generate_trajectory.py \
    --object sample \
    --num_viewpoints 163
# 출력: data/sample/trajectory/163/trajectory.csv

# 3단계: 시뮬레이션
/isaac-sim/python.sh scripts/3_simulation.py \
    --object sample \
    --num_viewpoints 163 \
    --visualize_spheres
```

## 스크립트 설명

### 1_create_viewpoint.py

멀티 머티리얼 메시에서 검사 대상 표면을 추출하고 뷰포인트 생성

**주요 기능**:
- RGB 색상 기반 머티리얼 선택
- 뷰포인트 개수 자동 계산 (카메라 FOV, 작업 거리, 오버랩 비율 고려)
- Poisson 디스크 샘플링

**인자**:
```bash
--object         # 객체 이름 (필수)
--material-rgb   # 대상 머티리얼 RGB "R,G,B" (필수)
--visualize      # Open3D 시각화 (옵션)
```

---

### 2_generate_trajectory.py

모든 뷰포인트를 최적 순서로 방문하는 충돌 없는 로봇 궤적 계산

**처리 과정**:
1. IK 계산 (EAIK, 32 seeds)
2. 충돌 필터링
3. GTSP 최적화 (k-NN + DP)
4. 보간 및 충돌 검사
5. 재계획 (MotionGen, 최대 5회 시도, 10초 타임아웃)

**인자**:
```bash
--object          # 객체 이름 (필수)
--num_viewpoints  # 뷰포인트 개수 (필수)
--knn             # k-NN 이웃 수 (기본값: 5)
--lambda-rot      # 회전 비용 가중치 (기본값: 1.0)
--visualize       # 궤적 시각화 (옵션)
```

**출력 통계 예시**:
```
Waypoints: 146
Interpolated: 1484
Collisions found: 4
Replanning: 3/4 succeeded (75.0%)
```

---

### 3_simulation.py

Isaac Sim에서 궤적 실행

**인자**:
```bash
--object            # 객체 이름 (필수)
--num_viewpoints    # 뷰포인트 개수 (필수)
--visualize_spheres # 로봇 충돌 구체 표시 (옵션)
--debug             # 목표 웨이포인트 표시 (옵션)
--headless          # 헤드리스 모드: native 또는 websocket (옵션)
```

## 디렉토리 구조

```
data/{object}/
├── mesh/
│   ├── source.obj        # 입력 메시 (1, 2단계)
│   └── source.mtl        # 머티리얼 정의
├── viewpoint/
│   └── {num}/
│       └── viewpoints.h5 # 1단계 출력
└── trajectory/
    └── {num}/
        └── trajectory.csv # 2단계 출력
```

## 설정

`new_common/config.py`에서 모든 파라미터 설정:

- **카메라**: FOV 41×30mm, 작업 거리 110mm, 오버랩 50%
- **IK**: Seeds 32개, 위치 임계값 0.005m, 회전 임계값 0.05 rad
- **충돌**: 최대 관절 스텝 1.0°
- **재계획**: 최대 60회 시도, 10초 타임아웃

## 문제 해결

**"MTL file not found"**
- `source.obj`가 `source.mtl`과 함께 있는지 확인

**"Replanning: 0/N succeeded"**
- `new_common/config.py`에서 `REPLAN_TIMEOUT`, `REPLAN_MAX_ATTEMPTS` 증가
- 로봇 관절 한계 확인

**"Input mesh not found"**
- 디렉토리 구조 확인: `data/{object}/mesh/source.obj`

## 성능

일반적인 실행 시간 (163 뷰포인트, UR20):
- 1단계: ~2초
- 2단계: ~4초
- 3단계: 실시간