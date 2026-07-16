#include "sdk.hpp"

// ==========================================================
// 생성자
// ==========================================================
SDK_Motion::SDK_Motion()
{
    current_seg_idx_ = 0;
    current_tick_ = 0;
    current_motion_id_ = -1;

    current_v_.assign(NUMBER_OF_JOINTS, 0.0);
    current_a_.assign(NUMBER_OF_JOINTS, 0.0);

    define_motions();
}


// ==========================================================
// 시간을 200Hz 기준 0.005초 단위로 보정
// ==========================================================
double SDK_Motion::Snap_Time(double seconds) const
{
    const double dt = 1.0 / HZ; // 200Hz = 0.005초

    if (!std::isfinite(seconds)) // 만약 seconds가 유한하지 않다면 (NaN 또는 Inf)
    {
        return dt; // 기본 시간 단위로 설정
    }

    if (seconds < dt) // 만약 seconds가 0.005초보다 작다면
    {
        return dt; // 기본 시간 단위로 설정
    }

    // 가장 가까운 0.005초 배수로 반올림해서 반환
    return std::round(seconds / dt) * dt;
}


// ==========================================================
// 모션 존재 확인
// ==========================================================
bool SDK_Motion::Has_Motion(int motion_id) const
{
    return motion_library_.find(motion_id) != motion_library_.end();
    //motion_library_ 맵에서 motion_id가 존재하는지 확인
    // 존재하면 true, 없으면 false 반환
}


// ==========================================================
// 현재 움직이는 중인지 확인
// ==========================================================
bool SDK_Motion::Is_Moving() const
{
    return !active_trajectory_.empty() &&
           current_seg_idx_ < static_cast<int>(active_trajectory_.size());
    // active_trajectory_가 비어있지 않고, 현재 세그먼트 인덱스가 전체 세그먼트 수보다 작으면 움직이는 중으로 판단
    // 즉, active_trajectory_에 아직 실행할 세그먼트가 남아있으면 true 반환
}



// ==========================================================
// 모션 정의
// ==========================================================
void SDK_Motion::define_motions()
{
    // Dynamixel raw 0~4095 값을 radian으로 변환
    //
    // raw 2048 = 0 rad
    // raw가 2048보다 크면 양수
    // raw가 2048보다 작으면 음수
    auto R = [](int raw_value)
    {
        return (raw_value - 2048.0) * (PI / 2048.0);
    };


    // ----------------------------------------------------------
    // 모션 0번: 기본 자세
    // ----------------------------------------------------------
    MotionSequence pose_default;
    // 헤더 파일에서 선언된 MotionSequence는 모션 시퀀스를 나타내는 구조체
    // poses, durations, blends를 포함합니다.

    pose_default.poses =
    {
        {
            R(2075), R(1372), R(2225), R(2223), R(2054), R(2069),
            R(2753), R(1729), R(1465), R(2043), R(2045), R(2110),
            R(2058), R(1513), R(1007), R(1960), R(2093), R(2438),
            R(3093), R(2148), R(1989), R(1963)
        }
    };

    // 포즈가 1개뿐이므로 내부 duration은 없음 >> 아니면 Trajectory에서 false 반환
    pose_default.durations = {};

    // 기본 자세는 도착 후 정지
    pose_default.blends = {BlendType::Stop};

    // 모션 ID 0번에 기본 자세 등록
    motion_library_[0] = pose_default;


    // ----------------------------------------------------------
    // 모션 1번: pickup 예시
    // ----------------------------------------------------------
    MotionSequence motion_pickup;

    motion_pickup.poses =
    {
        {
            R(2099), R(780), R(3006), R(2412), R(2092), R(2049),
            R(3390), R(949), R(1343), R(1990), R(2045), R(2106),
            R(2064), R(1631), R(1001), R(1959), R(2089), R(2438),
            R(3105), R(2148), R(1993), R(1963)
        },
        {
            R(2100), R(784), R(3023), R(2405), R(2100), R(2046),
            R(3397), R(943), R(1347), R(1989), R(2051), R(2078),
            R(2059), R(1631), R(1005), R(1694), R(2093), R(2437),
            R(3101), R(2417), R(1948), R(1963)
        },
        {
            R(2100), R(784), R(3023), R(2405), R(2100), R(2046),
            R(3397), R(943), R(1347), R(1989), R(2051), R(1209),
            R(2059), R(1023), R(1764), R(1793), R(2081), R(2437),
            R(3101), R(2417), R(1948), R(1963)
        },
        {
            R(2116), R(560), R(3245), R(2433), R(2126), R(2007),
            R(3559), R(729), R(1269), R(1999), R(2074), R(1210),
            R(2051), R(1023), R(1767), R(1796), R(2082), R(2437),
            R(3099), R(2414), R(1946), R(1964)
        },
        {
            R(2115), R(561), R(3246), R(2433), R(2128), R(2006),
            R(3559), R(728), R(1269), R(2001), R(2075), R(1205),
            R(2051), R(1458), R(1931), R(1826), R(2033), R(2437),
            R(3097), R(2410), R(1946), R(1964)
        },
        {
            R(2115), R(562), R(3246), R(2435), R(2130), R(2008),
            R(3560), R(728), R(1269), R(2003), R(2077), R(1208),
            R(2052), R(1446), R(895), R(1683), R(2149), R(2437),
            R(3093), R(2404), R(1944), R(1964)
        }
    };

    // 중요:
    // 포즈가 6개면 내부 duration은 5개여야 함.
    // P1 -> P2
    // P2 -> P3
    // P3 -> P4
    // P4 -> P5
    // P5 -> P6
    // 현재 자세 -> P1 시간은 SelectMotion(1, transition_time)에서 따로 넣음.
    motion_pickup.durations =
    {
        1.0,
        1.0,
        1.0,
        1.0,
        1.0
    };

    // 각 포즈에 도착했을 때 Stop/Smooth 선택
    //
    // Smooth:
    //   해당 포즈에서 멈추지 않고 다음 포즈로 이어짐.
    //   속도는 앞뒤 포즈 평균속도로 계산됨.
    //
    // Stop:
    //   해당 포즈 끝에서 속도 0, 가속도 0으로 멈춤.
    //
    // 예:
    //   P3에서 0.5초 대기하고 싶으면
    //   poses = {P1, P2, P3, P3, P4}
    //   durations 중 P3 -> P3 시간을 0.5로 주면 됨.
    motion_pickup.blends =
    {
        BlendType::Smooth, // P1
        BlendType::Smooth, // P2
        BlendType::Stop,   // P3
        BlendType::Smooth, // P4
        BlendType::Smooth, // P5
        BlendType::Stop    // P6, 마지막 포즈는 최종 정지
    };

    // 모션 ID 1번에 pickup 모션 등록
    motion_library_[1] = motion_pickup;
}

// ==========================================================
// 5차 다항식 계수 계산
// ==========================================================
void SDK_Motion::calculate_coefficients(
    const std::vector<double>& q0,
    const std::vector<double>& qf,
    const std::vector<double>& v0,
    const std::vector<double>& vf,
    const std::vector<double>& a0,
    const std::vector<double>& af,
    double T,
    TrajectorySegment& segment
)
{
    // 시간을 0.005초 단위로 보정
    T = Snap_Time(T);

    segment.T = T;
    segment.total_ticks = std::max(1, static_cast<int>(std::round(T * HZ)));

    segment.c0.assign(NUMBER_OF_JOINTS, 0.0);
    segment.c1.assign(NUMBER_OF_JOINTS, 0.0);
    segment.c2.assign(NUMBER_OF_JOINTS, 0.0);
    segment.c3.assign(NUMBER_OF_JOINTS, 0.0);
    segment.c4.assign(NUMBER_OF_JOINTS, 0.0);
    segment.c5.assign(NUMBER_OF_JOINTS, 0.0);

    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    for (int i = 0; i < NUMBER_OF_JOINTS; ++i)
    {
        double dq = qf[i] - q0[i];
        // dq = 최종 포즈 qf[i]와 초기 포즈 q0[i]의 차이, 즉 이동해야 할 관절 각도 변화량
        // 음수가 나오면 해당 관절은 반대 방향으로 회전해야 함을 의미

        // 5차 다항식:
        // q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
        // 조건:
        // t=0 에서 q0, v0, a0
        // t=T 에서 qf, vf, af
        segment.c0[i] = q0[i];
        segment.c1[i] = v0[i];
        segment.c2[i] = a0[i] / 2.0;

        segment.c3[i] =
            (
                10.0 * dq
                - (6.0 * v0[i] + 4.0 * vf[i]) * T
                - (1.5 * a0[i] - 0.5 * af[i]) * T2
            ) / T3;

        segment.c4[i] =
            (
                -15.0 * dq
                + (8.0 * v0[i] + 7.0 * vf[i]) * T
                + (1.5 * a0[i] - af[i]) * T2
            ) / T4;

        segment.c5[i] =
            (
                6.0 * dq
                - 3.0 * (v0[i] + vf[i]) * T
                - (0.5 * a0[i] - 0.5 * af[i]) * T2
            ) / T5;
    }
}

// ==========================================================
// 모션 궤적 생성
// ==========================================================
bool SDK_Motion::Generate_Trajectory(
// 모션 생성이 성공하면 true, 실패하면 false 반환
    int motion_id, // 실행할 모션 ID
    double* current_pose, // 현재 관절각 배열 (nullptr이면 오류)
    double transition_time // 현재 자세에서 첫 번째 포즈까지 이동하는 시간(초)
)
{
    if (current_pose == nullptr) // 만약 현재 관절각 배열이 nullptr이면
    {
        std::cout << "[SDK] current_pose가 nullptr입니다." << std::endl;
        return false; // false 반환
    }

    if (!Has_Motion(motion_id)) // 만약 motion_id가 존재하지 않으면
    {
        std::cout << "[SDK] 존재하지 않는 모션 ID: "
                  << motion_id << std::endl;
        return false; // false 반환
    }
    // 이 부분은 sdk.cpp에서 처리할 수도 있지만,
    // main.cpp 또는 callback.cpp에서 IsMoving() 함수를 이용해 처리하는 방향으로 둠.
    //
    // if (Is_Moving())
    // {
    //     std::cout << "[SDK] 현재 모션 실행 중이라 새 명령 무시: ID "
    //               << motion_id << std::endl;
    //     return false;
    // }

    const auto& seq = motion_library_.at(motion_id);
    // motion_library_ 맵에서 motion_id에 해당하는 MotionSequence를 가져옴

    if (seq.poses.empty()) // 만약 해당 모션의 poses가 비어있으면
    {
        std::cout << "[SDK] 모션 ID " << motion_id
                  << "에 포즈가 없습니다." << std::endl;
        return false; // false 반환
    }

    for (size_t p = 0; p < seq.poses.size(); ++p) // 각 포즈마다 관절 개수 확인
    {
        if (seq.poses[p].size() != NUMBER_OF_JOINTS) // 만약 포즈의 관절 개수가 NUMBER_OF_JOINTS와 다르면
        {
            std::cout << "[SDK] 모션 ID " << motion_id
                      << "의 " << p << "번 포즈 관절 개수 오류. "
                      << "현재 개수: " << seq.poses[p].size()
                      << ", 필요 개수: " << NUMBER_OF_JOINTS
                      << std::endl;
            return false; // false 반환
        }
    }

    // 포즈가 N개면 내부 duration은 N-1개여야 함.
    // current_pose -> 첫 포즈 시간은 transition_time으로 따로 받기 때문.

    if (seq.poses.size() >= 2 && // poses가 2개 이상이면 durations 개수 확인
        seq.durations.size() != seq.poses.size() - 1) // 만약 durations 개수가 poses 개수 - 1과 다르면
    {
        std::cout << "[SDK] 모션 ID " << motion_id
                  << " duration 개수 오류. "
                  << "poses=" << seq.poses.size()
                  << ", durations=" << seq.durations.size()
                  << ", 필요한 durations=" << seq.poses.size() - 1
                  << std::endl;
        return false; // false 반환
    }

    if (seq.poses.size() == 1 && !seq.durations.empty()) // 만약 poses가 1개인데 durations가 비어있지 않으면
    {
        std::cout << "[SDK] 모션 ID " << motion_id
                  << "는 포즈가 1개이므로 durations는 비어 있어야 합니다."
                  << std::endl;
        return false; // false 반환
    }

    // blends는 비어 있으면 기본값으로 처리 가능.
    // 하지만 넣을 거면 poses 개수와 맞춰야 함.
    if (!seq.blends.empty() && seq.blends.size() != seq.poses.size()) // 만약 blends가 비어있지 않고 poses 개수와 다르면
    {
        std::cout << "[SDK] 모션 ID " << motion_id
                  << " blends 개수 오류. "
                  << "poses=" << seq.poses.size()
                  << ", blends=" << seq.blends.size()
                  << ", 필요한 blends=" << seq.poses.size()
                  << std::endl;
        return false; // false 반환
    }

    active_trajectory_.clear(); // 이전에 생성된 궤적을 초기화
    current_motion_id_ = motion_id; // 현재 모션 ID를 업데이트

    // ----------------------------------------------------------
    // 1. waypoint 구성 (waypoints 개수 = poses 개수 + 1)
    // ----------------------------------------------------------
    //
    // 모션 내부 포즈가 P1, P2, P3, P4이면
    // waypoints[0] = 현재 자세
    // waypoints[1] = P1
    // waypoints[2] = P2
    // waypoints[3] = P3
    // waypoints[4] = P4
    std::vector<std::vector<double>> waypoints;

    waypoints.push_back(
        std::vector<double>(
            current_pose,
            current_pose + NUMBER_OF_JOINTS
        )
    );

    for (const auto& p : seq.poses)
    {
        waypoints.push_back(p);
    }

    // ----------------------------------------------------------
    // 2. 구간 시간 구성
    // ----------------------------------------------------------
    //
    // durations[0] = 현재 자세 -> P1
    // durations[1] = P1 -> P2
    // durations[2] = P2 -> P3
    // ...
    // 위에서 정의한 durations는 모션 내부 포즈 간 이동 시간만 포함되어 있음
    // 따라서 현재 자세에서 첫 번째 포즈까지 이동하는 시간을 transition_time으로 따로 넣어야 함.
    std::vector<double> durations;

    durations.push_back(Snap_Time(transition_time));
    // durations[0] = 현재 자세 -> P1 이동 시간 (transition_time)
    // durations의 개수가 하나 증가함.
    for (double d : seq.durations) // 모션 내부 포즈 간 이동 시간들을 durations에 추가
    {
        durations.push_back(Snap_Time(d));
    }

    const int M = static_cast<int>(waypoints.size()); // M = waypoints 개수, 즉 모션 전체 포즈 개수 + 1

    if (durations.size() != waypoints.size() - 1) // 만약 durations 개수가 waypoints 개수 - 1과 다르면
    {
        std::cout << "[SDK] 내부 duration 구성 오류. "
                  << "waypoints=" << waypoints.size()
                  << ", durations=" << durations.size()
                  << std::endl;
        return false; // false 반환
    }

    // ----------------------------------------------------------
    // 3. 각 waypoint의 속도 V, 가속도 A 설정
    // ----------------------------------------------------------
    //
    // V[i] = waypoints[i]를 지날 때의 속도
    // A[i] = waypoints[i]를 지날 때의 가속도
    //
    // 시작점:
    //   정지 상태에서 새 모션 시작한다고 가정.
    //   속도 0, 가속도 0.
    //
    // 중간 포즈:
    //   Smooth면 앞뒤 포즈 평균속도로 통과.
    //   Stop이면 속도 0, 가속도 0.
    //
    // 마지막 포즈:
    //   모션 끝나고 판단해야 하므로 항상 속도 0, 가속도 0.

    std::vector<std::vector<double>> V(
        M,
        std::vector<double>(NUMBER_OF_JOINTS, 0.0)
    );
    // 각 웨이포인트에서의 속도 벡터 초기화, M개의 웨이포인트마다 NUMBER_OF_JOINTS개의 관절 속도
    std::vector<std::vector<double>> A(
        M,
        std::vector<double>(NUMBER_OF_JOINTS, 0.0)
    );
    // waypoints[0]은 현재 자세라서 V=0, A=0 유지.
    // waypoints[M-1]은 마지막 포즈라서 V=0, A=0 유지.

    // 따라서 중간 waypoint만 처리.
    for (int wp = 1; wp < M - 1; ++wp)
    {
        // waypoints[1] = poses[0]
        // waypoints[2] = poses[1]
        // 따라서 pose_idx = wp - 1
        const int pose_idx = wp - 1;

        BlendType blend = BlendType::Smooth; // 기본값은 Smooth

        if (!seq.blends.empty()) // 만약 blends가 비어있지 않으면
        {
            blend = seq.blends[pose_idx]; // 해당 포즈의 blend 타입 가져오기
        }

        if (blend == BlendType::Smooth) // Smooth이면
        {
            // Smooth:
            // 해당 포즈에서 멈추지 않고 지나감.
            // 속도는 이전 포즈와 다음 포즈를 기준으로 한 평균속도.
            // V[wp] = (next - prev) / (prev_time + next_time)
            double denom = durations[wp - 1] + durations[wp]; // denom = (이전 포즈 -> 현재 포즈 시간) + (현재 포즈 -> 다음 포즈 시간)

            if (denom < 1.0 / HZ) // 만약 두 구간의 합이 0.005초보다 작으면
            {
                denom = 1.0 / HZ; // 최소 0.005초로 설정 (속도가 너무 커지는 것을 방지)
            }

            for (int j = 0; j < NUMBER_OF_JOINTS; ++j) // 각 관절마다 속도 계산
            {
                V[wp][j] =
                    (waypoints[wp + 1][j] - waypoints[wp - 1][j])
                    / denom; // 이전 포즈와 다음 포즈의 차이를 두 구간 시간의 합으로 나누어 평균속도 계산
                A[wp][j] = 0.0; // 가속도는 0으로 설정
            }
        }

        else // Stop이면
        {
            // 각 포즈마다 속도 0, 가속도 0.
            for (int j = 0; j < NUMBER_OF_JOINTS; ++j)
            {
                V[wp][j] = 0.0;
                A[wp][j] = 0.0;
            }
        }
    }

    // ----------------------------------------------------------
    // 4. 각 구간을 5차 다항식 segment로 변환
    // ----------------------------------------------------------
    for (int i = 0; i < M - 1; ++i)
    {
        TrajectorySegment seg; // 각 구간을 나타내는 TrajectorySegment 구조체 생성 (hpp에서 정의됨)

        calculate_coefficients(
            waypoints[i],
            waypoints[i + 1],
            V[i],
            V[i + 1],
            A[i],
            A[i + 1],
            durations[i],
            seg
            // seg에 계산된 5차 다항식 계수 저장
        );

        active_trajectory_.push_back(seg); // active_trajectory_에 각 구간 segment 추가
    }

    current_seg_idx_ = 0; // 현재 세그먼트 인덱스를 0으로 초기화 (첫 번째 구간부터 시작)
    current_tick_ = 0; // 현재 tick을 0으로 초기화 (첫 번째 tick부터 시작)

    std::cout << "[SDK] 모션 ID " << motion_id
              << " 실행 준비 완료. segment 개수: "
              << active_trajectory_.size()
              << std::endl;

    return true; // 모션 궤적 생성 성공해서 true 반환
}

// ==========================================================
// 다음 tick 목표각 계산
// ==========================================================
bool SDK_Motion::Get_Next_Tick(double* All_Theta)
// All_Theta: 다음 tick 목표각을 저장할 배열 (nullptr이면 오류)
// Get_Next_Tick()는 현재 segment와 tick을 기준으로 다음 tick 목표각을 계산하고 All_Theta에 저장
// 반환값: true = 다음 tick 목표각 계산 성공, false = 모션 종료 또는 오류
{
    if (All_Theta == nullptr) // 만약 All_Theta가 nullptr이면
    {
        return false; // false 반환
    }

    if (!Is_Moving()) // 만약 현재 모션이 진행 중이 아니면
    {
        current_v_.assign(NUMBER_OF_JOINTS, 0.0);
        current_a_.assign(NUMBER_OF_JOINTS, 0.0);
        return false; // false 반환
        // 모션 시작 순간 current_tick_ = 0이어도 active_trajectory_가 있으니 Is_Moving()는 true 반환
        // 하지만 모션이 끝나면 active_trajectory_가 비어있으므로 Is_Moving()는 false 반환
    }

    const auto& seg = active_trajectory_[current_seg_idx_]; // 현재 segment 가져오기

    // current_tick_는 0부터 시작하지만,
    // 출력은 다음 tick 목표값을 주는 방식으로 함.
    //
    // 200Hz 기준:
    // current_tick_ = 0 -> t = 0.005
    // current_tick_ = 1 -> t = 0.010
    // ...
    // 마지막 tick -> t = T
    //
    // 이렇게 하면 마지막 포즈값을 정확히 찍고 끝남.

    double t = static_cast<double>(current_tick_ + 1) / HZ; // 다음 tick 목표각을 계산하기 위해 현재 tick + 1을 사용하여 시간 t 계산

    if (t > seg.T) // 만약 t가 segment의 총 시간 T보다 크면
    {
        t = seg.T; // 마지막 tick에서 segment의 총 시간 T로 보정 (마지막 tick에서 segment의 총 시간 T로 보정)
    }

    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) // 각 관절마다 위치, 속도, 가속도 계산
    {
        // 위치
        All_Theta[i] =
            seg.c0[i]
            + seg.c1[i] * t
            + seg.c2[i] * t2
            + seg.c3[i] * t3
            + seg.c4[i] * t4
            + seg.c5[i] * t5;

        // 속도
        current_v_[i] =
            seg.c1[i]
            + 2.0 * seg.c2[i] * t
            + 3.0 * seg.c3[i] * t2
            + 4.0 * seg.c4[i] * t3
            + 5.0 * seg.c5[i] * t4;

        // 가속도
        current_a_[i] =
            2.0 * seg.c2[i]
            + 6.0 * seg.c3[i] * t
            + 12.0 * seg.c4[i] * t2
            + 20.0 * seg.c5[i] * t3;
    }
    // All_Theta에 다음 tick 목표각 계산 완료

    current_tick_++; // 현재 tick 증가

    // 현재 segment가 끝났으면 다음 segment로 이동
    if (current_tick_ >= seg.total_ticks) // 만약 현재 tick이 segment의 총 tick 수보다 크거나 같으면
    {
        current_seg_idx_++; // 다음 segment로 이동
        current_tick_ = 0; // 다음 segment의 첫 번째 tick으로 초기화 (다음 포즈의 첫 번째 tick부터 시작)

        // 전체 모션 종료
        if (current_seg_idx_ >= static_cast<int>(active_trajectory_.size()))
        // 만약 현재 segment 인덱스가 전체 active_trajectory_의 크기보다 크거나 같으면
        {
            active_trajectory_.clear(); // active_trajectory_를 비워서 모션 종료 상태로 만듦

            current_seg_idx_ = 0; // 현재 segment 인덱스를 0으로 초기화
            current_tick_ = 0; // 현재 tick을 0으로 초기화
            current_motion_id_ = -1; // 현재 모션 ID를 -1로 초기화 (모션 종료 상태)

            current_v_.assign(NUMBER_OF_JOINTS, 0.0); // 현재 속도를 0으로 초기화
            current_a_.assign(NUMBER_OF_JOINTS, 0.0); // 현재 가속도를 0으로 초기화
        }
    }
    return true; // 다음 tick 목표각 계산 성공 (모션이 아직 진행 중이거나, 모션이 끝나서 active_trajectory_가 비워졌지만 정상적으로 종료된 경우)
}
