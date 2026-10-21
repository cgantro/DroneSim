#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief 로봇 Controller가 주고받는 명령과 상태의 공통 자료형을 정의한다.
 * @details 회전 관절 위치·속도는 [rad], [rad/s]이고 직선 관절은 [m], [m/s]다. TCP 위치는 [m], 시간은 [s]다.
 * TCP는 공구 끝에서 작업 위치와 방향을 나타내는 기준점이다.
 * 방향 quaternion은 회전을 네 숫자 [x,y,z,w]로 저장한다.
 * Controller가 실제로 보고한 TCP는 FK로 계산한 모델 ToolFrame과 별도다.
 * 이 자료형은 숫자의 단위와 구성을 정할 뿐 장치 연결이나 단위 변환을 수행하지 않는다.
 */
namespace grasplink::robotics
{

/** @brief RobotSpecification에 적힌 순서대로 각 관절 위치를 저장한다. 회전 관절은 [rad], 직선 관절은 [m]다. */
using JointVector = std::vector<double>;

/**
 * @brief 요청이 처리됐는지와 실패 종류를 나타낸다.
 * @details None은 성공, NotConnected는 연결 없음, InvalidCommand는 잘못된 요청값을 뜻한다.
 * Busy는 앞선 동작 때문에 요청을 받을 수 없음, Fault는 장치나 구현의 오류를 뜻한다.
 * Unsupported는 이 구현이 기능을 제공하지 않음, TransportError는 통신 실패를 뜻한다.
 * Unreachable은 IK의 보수적 도달 거리 밖이고 JointLimitReached는 관절 경계에 막힌 국소 계산이며 IkDidNotConverge는 반복 한도나 정체다.
 * 국소 IK 실패만으로 다른 시작 관절각에서도 해가 없다고 단정하지 않는다.
 * SelfCollision과 AttachedObjectCollision은 계획한 자세가 각각 로봇 링크끼리 또는 로봇에 붙은 물체와 충돌할 때 사용한다.
 * Fault code만으로 장비 보호 정지 여부를 판단할 수 없다.
 */
enum class ErrorCode
{
    None,           // 요청을 오류 없이 처리했다.
    NotConnected,   // 장치나 시뮬레이션 제어기가 연결되지 않았거나 동작 중 연결을 잃었다.
    InvalidCommand, // 관절 수, 허용 범위 또는 유한성 등 요청 값에 문제가 있다.
    Busy,           // 앞선 동작을 처리하고 있어 새 요청을 지금 받을 수 없다.
    Fault,          // 장비 또는 시뮬레이션 제어기에서 오류가 발생했다.
    Unsupported,    // 공통 인터페이스에는 있지만 현재 제어기에서 제공하지 않는 기능이다.
    TransportError, // TCP, Serial 또는 Modbus 연결에서 통신이 실패했다.
    Unreachable,    // IK가 링크 길이의 보수적 거리 상한 밖인 목표를 확인했다.
    JointLimitReached, // IK가 관절 한계 때문에 국소 오차를 더 줄이지 못했다.
    IkDidNotConverge, // IK가 반복 한도나 수치 정체로 해를 찾지 못했다. 다른 시작각에서 해가 없다는 뜻은 아니다.
    EnvironmentContact, // 다음 Robot 또는 Gripper 목표 자세가 Environment와 겹쳐 물리 반영 전에 직전 관절 자세로 돌아갔다.
    SelfCollision, // 계획한 관절 자세에서 로봇 링크끼리 충돌한다.
    AttachedObjectCollision, // 계획한 관절 자세에서 로봇에 붙은 물체가 충돌한다.
    Cancelled // 진행 중인 소프트웨어 계획 또는 동작 요청이 취소되었다.
};

/**
 * @brief 장치 또는 Simulation 구현이 요청을 처리한 결과와 선택적 설명을 반환한다.
 * @details 성공은 요청을 받아들였다는 뜻일 수 있으며 목표 위치 도달이나 동작 완료를 보장하지 않는다.
 * 소프트웨어 정지 요청은 장비의 보호 정지(protective stop)나 비상 정지(E-Stop)를 보장하지 않는다.
 */
struct Result
{
    /// code가 None이면 성공이다. 실패한 경우 message에 원인을 설명하는 추가 내용을 담을 수 있다.
    ErrorCode code = ErrorCode::None;

    /// 오류 원인을 사람이 읽을 수 있도록 설명하는 선택 메시지다.
    std::string message;

    [[nodiscard]] bool Ok() const noexcept { return code == ErrorCode::None; }
    explicit operator bool() const noexcept { return Ok(); }

    static Result Success() { return {}; }
};

/** @brief TCP 위치 [m]와 방향을 묶는다. quaternion 성분은 [x,y,z,w] 순서이며 기준 좌표계와 유효 여부는 별도 상태에서 확인한다. */
struct CartesianPose
{
    std::array<double, 3> positionMeters{};

    /// 방향 quaternion 성분은 [x,y,z,w] 순서다. (0,0,0,1)은 회전이 없는 상태다.
    std::array<double, 4> orientationXyzw{0.0, 0.0, 0.0, 1.0};
};

/** @brief 관절별 목표 위치와 제어기가 적용할 속도·가속도 비율을 담는다. */
struct JointMoveCommand
{
    /// RobotSpecification에 적힌 순서의 목표 위치다. 회전 관절은 [rad], 직선 관절은 [m]다.
    JointVector targetPositions;

    /// 제어기마다 허용 범위와 해석이 다를 수 있다. 시뮬레이션은 (0,1] 값을 모델의 관절별 최대 속도에 곱한다.
    double velocityScale = 1.0;

    /// 제어기마다 해석이 다를 수 있다. 시뮬레이션은 최대 속도까지 가속하는 시간의 비율로 사용한다.
    double accelerationScale = 1.0;

};

// TCP는 공구 끝의 작업 기준점이다. TCP 목표에서 관절 목표를 구하는 역기구학(IK)과 경로 실행이 있어야 직선 이동할 수 있다.
/** @brief 직선 이동 명령의 기본 속도와 가속도 요청값이다. */
namespace linear_motion_defaults
{
inline constexpr double linearVelocityMetersPerSecond = 0.25;
inline constexpr double angularVelocityRadiansPerSecond = 0.5;
inline constexpr double linearAccelerationMetersPerSecondSquared = 12.0;
inline constexpr double angularAccelerationRadiansPerSecondSquared = 60.0;
}

/** @brief TCP가 도착할 위치·방향과 직선 이동의 최대 속도·가속도를 요청한다. 관절 목표를 계산하는 IK와 경로 실행 기능이 제어 구현에 있어야 처리할 수 있다. */
struct LinearMoveCommand
{
    /// 도착할 TCP 위치 [m]와 방향 quaternion [x,y,z,w]다. 기준 좌표계는 공통 형식만으로 정하지 않는다.
    CartesianPose targetPose{};

    /// 직선 경로를 실행할 때 허용할 TCP 이동 속도 상한 [m/s]다.
    double maxLinearVelocityMetersPerSecond = linear_motion_defaults::linearVelocityMetersPerSecond;

    /// 직선 경로를 실행할 때 허용할 TCP 회전 속도 상한 [rad/s]다.
    double maxAngularVelocityRadiansPerSecond = linear_motion_defaults::angularVelocityRadiansPerSecond;

    /// 직선 경로를 실행할 TCP 선가속도 상한 [m/s²]다.
    double maxLinearAccelerationMetersPerSecondSquared = linear_motion_defaults::linearAccelerationMetersPerSecondSquared;

    /// 직선 경로를 실행할 TCP 각가속도 상한 [rad/s²]다.
    double maxAngularAccelerationRadiansPerSecondSquared = linear_motion_defaults::angularAccelerationRadiansPerSecondSquared;
};

/**
 * @brief TCP가 지정한 여러 자세를 순서대로 지나가도록 하나의 경로를 요청한다.
 * @details 각 자세 사이에서는 직선 위치와 최단 회전 경로를 사용한다. 전체 경로는 하나의 가속·감속 프로파일로 실행한다.
 */
struct LinearPathMoveCommand
{
    /// TCP 자세는 로봇 도구 끝의 위치와 방향을 함께 나타낸다. 이 목록의 자세를 입력 순서대로 통과한다.
    std::vector<CartesianPose> targetPoses;
    /// 경로 전체에서 허용하는 TCP 직선 이동 속도 상한 [m/s]이다.
    double maxLinearVelocityMetersPerSecond = linear_motion_defaults::linearVelocityMetersPerSecond;
    /// 경로 전체에서 허용하는 TCP 회전 속도 상한 [rad/s]이다.
    double maxAngularVelocityRadiansPerSecond = linear_motion_defaults::angularVelocityRadiansPerSecond;
    /// 경로 전체에서 허용하는 TCP 직선 가속도 상한 [m/s²]이다.
    double maxLinearAccelerationMetersPerSecondSquared = linear_motion_defaults::linearAccelerationMetersPerSecondSquared;
    /// 경로 전체에서 허용하는 TCP 회전 가속도 상한 [rad/s²]이다.
    double maxAngularAccelerationRadiansPerSecondSquared = linear_motion_defaults::angularAccelerationRadiansPerSecondSquared;
};

/** @brief Controller가 연결되어 있는지, 움직이는 중인지, 정지했는지를 나타낸다. */
enum class RobotMode
{
    Disconnected, // 제어 구현 연결 또는 초기화 전.
    Idle,         // 연결은 되었고 실행 중인 동작 명령은 없다.
    Moving,       // 관절이 목표 위치를 향해 움직이는 중이다.
    Planning,     // 관절 자세는 유지하며 경로 계획을 분할 실행하는 중이다.
    Stopped,      // 소프트웨어 Stop 요청에 따라 동작이 멈춘 상태다.
    Fault         // 오류가 있어 정상적인 제어를 할 수 없다.
};

/**
 * @brief 한 시점에 Controller가 보고한 관절과 TCP 상태를 값으로 보관한다.
 * @details 관절 값은 RobotSpecification 순서로 저장한다. 회전 관절은 [rad], 직선 관절은 [m]를 쓰며,
 * 속도는 각각 [rad/s], [m/s]다. TCP 위치는 [m]다.
 * valid는 관절을 포함한 전체 상태 복사본을 쓸 수 있는지 나타낸다.
 * tcpPoseValid는 TCP 위치·방향만 유효한지 나타내므로 TCP가 없어도 관절 상태는 유효할 수 있다.
 * errorCode는 공통 실패 분류를 사용하고, 장치별 원본 오류값은 별도로 해석한다.
 * 공통 오류 분류만으로 실제 장비 보호 정지 여부를 판정하지 않는다.
 */
struct RobotState
{
    /// RobotSpecification 순서에 맞춘 현재 관절 위치다. 회전 관절은 [rad], 직선 관절은 [m]다.
    JointVector jointPositions;

    /// RobotSpecification 순서에 맞춘 현재 관절 속도다. 회전 관절은 [rad/s], 직선 관절은 [m/s]다.
    JointVector jointVelocities;

    /// Controller가 보고한 TCP 위치·방향이다. Simulation은 설정한 공구 offset과 관절 FK에서 계산하고 실제 장치는 장치 feedback을 사용한다. tcpPoseValid를 별도로 확인한다.
    CartesianPose tcpPose{};

    RobotMode mode = RobotMode::Disconnected;

    /// Controller가 보고한 오류 분류다. Gripper의 장치별 원본 fault code와 달리 공통 ErrorCode 값이다.
    ErrorCode errorCode = ErrorCode::None;

    /// TCP 위치·방향 feedback만 유효한지 나타낸다. 관절값 유효성과 별개이며 Simulation에서는 모델로 계산한 유효성을 뜻한다.
    bool tcpPoseValid = false;

    /// 이 상태 복사본의 feedback을 현재 상태로 사용해도 되는지 나타낸다. TCP 유효성과 별개다.
    bool valid = false;
};

/**
 * @brief Gripper Controller에 보낼 위치·속도·힘 요청값을 담는다.
 * @details 각 필드는 장치 프로토콜의 원본 정수 code [0,255]다.
 * Code는 미터, [mm/s], [N] 같은 물리량이 아니며 공통 환산식도 없다.
 * 실제 장치나 Simulation이 code를 해석해 관절 움직임을 정한다.
 */
struct GripperCommand
{
    /// 장치 프로토콜에 보낼 위치 code [0,255]다. 미터나 각도로 변환된 물리 위치가 아니다.
    std::uint8_t positionRequest = 0;

    /// 장치 프로토콜에 보낼 속도 code [0,255]다. 실제 이동 속도 [mm/s]는 제어 구현별 변환이 필요하다.
    std::uint8_t speedRequest = 255;

    /// 장치 프로토콜에 보낼 힘 code [0,255]다. 실제 힘 [N]은 제어 구현별 변환이 필요하다.
    std::uint8_t forceRequest = 128;
};

/** @brief 그리퍼 제어기가 움직임, 접촉 또는 요청 위치 도달 상태를 보고한다. */
enum class GripperObjectStatus : std::uint8_t
{
    Moving = 0,              // 접촉이나 목표 도달은 확인되지 않았다. 실제 이동 중인지는 mode와 goToActive도 확인한다.
    ContactWhileOpening = 1, // 열리는 방향으로 움직이다 물체에 닿았다고 보고했다.
    ContactWhileClosing = 2, // 닫히는 방향으로 움직이다 물체에 닿았다고 보고했다.
    AtRequestedPosition = 3  // backend가 요청 위치에 도달했다고 보고했다.
};

/**
 * @brief Gripper 제어 구현의 연결, 활성화, 이동, 정지 또는 오류 상태를 나타낸다.
 * @details objectStatus는 접촉 여부나 목표 위치 도달을 표현하고 mode는 연결과 명령 실행 상태를 표현한다.
 * 중간 위치에서 Stop한 경우를 목표에 도달한 상태와 구분하도록 Stopped를 별도로 둔다.
 */
enum class GripperMode : std::uint8_t
{
    Disconnected,
    Inactive,
    Idle,
    Moving,
    Stopped,
    Fault
};

/**
 * @brief 한 시점에 Controller가 보고한 Gripper 상태를 값으로 보관한다.
 * @details actualPosition과 currentRaw는 장치의 정수 code이며 물리 단위로 변환되지 않는다.
 * closureFraction은 열린 기준 0부터 닫힌 기준 1까지의 연속 비율이며 거리나 힘이 아니다.
 * 이 연속값으로 기구 회전을 계산하면 8-bit 위치 반올림을 피할 수 있다.
 * valid가 false이면 상태 복사본을 현재 feedback으로 사용하지 않는다.
 * 접촉, 보호 정지, 손가락 연동의 의미는 장치나 Simulation 구현별로 다르다.
 */
struct GripperState
{
    /// 연결, 활성화 및 명령 실행 상태다. 물체 접촉이나 목표 도달 분류는 objectStatus에 따로 기록한다.
    GripperMode mode = GripperMode::Disconnected;

    /// 제어 구현 또는 장치가 Gripper 구동부를 활성화했다고 보고했는지 나타낸다.
    bool activated = false;

    /// 제어 구현 또는 장치가 요청 위치로 이동하는 명령을 실행 중이라고 보고했는지 나타낸다.
    bool goToActive = false;

    /// 장치 프로토콜에서 받은 활성화 상태 원본 code다. 공통 단위로 변환하지 않는다.
    std::uint8_t activationStatus = 0;

    /// 이동 중, 열거나 닫으며 접촉, 요청 위치 도달 중 어떤 상태인지 분류한다.
    GripperObjectStatus objectStatus = GripperObjectStatus::AtRequestedPosition;

    /// 장치나 시뮬레이션 제어기가 보낸 오류 원본 code다. 값의 뜻은 제어기마다 다를 수 있다.
    std::uint8_t faultCode = 0;

    /// 장치가 되돌려 준 위치 요청 code다. 현재 실제 위치가 아니라 요청값의 확인용 복사본이다.
    std::uint8_t requestedPositionEcho = 0;

    /// 장치나 시뮬레이션 제어기가 보고한 위치 원본 code다. 미터 단위의 거리는 아니다.
    std::uint8_t actualPosition = 0;

    /// 장치가 보고한 전류 원본 code다. 시뮬레이션은 전류를 계산하지 않는다.
    std::uint8_t currentRaw = 0;

    /// currentRaw가 실제 제공된 전류 feedback인지 여부. Simulation은 전류를 계산하지 않는다.
    bool currentValid = false;

    /// 연속 개폐 위치 [0,1]. 0은 열린 기준, 1은 닫힌 기준이며 물리 거리·힘 단위가 아니다.
    double closureFraction = 0.0;

    /// closureFraction이 유한한 [0,1] 위치로 제공되었는지 여부.
    bool closureFractionValid = false;

    /// 이 상태 복사본의 feedback을 현재 상태로 사용해도 되는지 나타낸다.
    bool valid = false;
};

}
