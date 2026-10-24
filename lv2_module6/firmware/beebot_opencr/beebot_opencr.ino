// beebot OpenCR 펌웨어 — SBC(control/BeebotSystemHardware) ↔ OpenCR ↔ DYNAMIXEL 8개 + 내장 IMU
//   USB 시리얼 ASCII 줄 프로토콜 (module5 opencr_pan_tilt와 같은 방식, 단위는 SI):
//     SBC → OpenCR  C,<바퀴L rad/s>,<바퀴R rad/s>,<j1>,<j2>,<j3>,<j4>,<j5 rad>,<그리퍼 m>\n      (매 제어 주기)
//                        빈 필드(`,,` ·trailing comma · 모자란 뒷필드)는 "유지" — 팔·그리퍼는 명령 생략(현재 유지),
//                        바퀴는 정지(0). 전부 빈칸이면 바퀴 정지 + 팔 유지. 예: C,0.3,0.3,,,,,,
//                        (빈칸 없이 전부 숫자였던 기존 C와 호환 — SBC는 팔 명령이 없을 때(NaN) 팔 칸을 빈칸으로 보냄)
//     OpenCR → SBC  S,<바퀴L pos,vel>,<바퀴R pos,vel>,<j1 pos,vel,A>…<j5 …>,<그리퍼 m,m/s,A>,
//                        <qx,qy,qz,qw>,<gx,gy,gz rad/s>,<ax,ay,az m/s²>\n                         (STATE_PERIOD_MS마다)
//   관절 0rad = 서보 중앙(2048 = 180°). 바퀴 위치는 부팅 후 누적 rad.
//   토크: T,0 = 전체 해제(손으로 자세 잡기) · T,1 = 전체 켜기(현재 자세 목표) ·
//         T,<8필드> = 관절별(빈칸=지금 상태 유지, 1=켜기 0=끄기). 예: T,1,0,,,,,,
//   watchdog: 마지막 유효 C부터 COMMAND_TIMEOUT_MS 지나면 바퀴 속도 0, 팔·그리퍼는 현재 자세 유지 (한 번)
// TODO(김현하): SyncRead/SyncWrite로 바꿔 주기 올리기, 관절 부호·영점·그리퍼 변환 실측, 전류 한계 설정
#include <Dynamixel2Arduino.h>
#include <IMU.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

Dynamixel2Arduino dxl(Serial3, DXL_DIR_PIN);
cIMU imu;

constexpr size_t COMMAND_FIELDS = 2 + ARM_COUNT + 1;
char line[LINE_CAPACITY];
size_t line_length = 0;
bool motors_ready = false;
bool watchdog_tripped = true;  // 첫 C 전에는 움직이지 않음
uint32_t last_command_ms = 0;
uint32_t last_state_ms = 0;

float rpm_to_rad_s(float rpm) { return rpm * 2.0f * PI / 60.0f; }
float rad_s_to_rpm(float rad_s) { return rad_s * 60.0f / (2.0f * PI); }
float joint_rad_to_deg(float rad, float dir) { return dir * rad * 180.0f / PI + 180.0f; }
float deg_to_joint_rad(float deg, float dir) { return dir * (deg - 180.0f) * PI / 180.0f; }

bool setup_motor(uint8_t id, uint8_t mode)
{
    if (!dxl.ping(id))
        return false;
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, mode);
    if (mode != OP_VELOCITY)  // 시작 자세 = 현재 자세 (첫 명령 전에 튀지 않게)
        dxl.setGoalPosition(id, dxl.getPresentPosition(id, UNIT_DEGREE), UNIT_DEGREE);
    return dxl.torqueOn(id);
}

void hold_present_pose()
{
    dxl.setGoalVelocity(WHEEL_LEFT_ID, 0);
    dxl.setGoalVelocity(WHEEL_RIGHT_ID, 0);
    for (size_t i = 0; i < ARM_COUNT; ++i)
        dxl.setGoalPosition(ARM_IDS[i], dxl.getPresentPosition(ARM_IDS[i], UNIT_DEGREE), UNIT_DEGREE);
    dxl.setGoalPosition(GRIPPER_ID, dxl.getPresentPosition(GRIPPER_ID, UNIT_DEGREE), UNIT_DEGREE);
}

bool parse_fields(char *text, float *out, size_t count)
{
    // 필드 수는 유연하게: 모자란 뒷 필드는 빈칸(유지)으로 채운다.
    //   C,0.3,0.3            → 바퀴만 (팔·그리퍼 유지)
    //   C,0.3,0.3,,,,,       → 위와 같음 (빈칸 명시)
    //   C,0.3,0.3,+1(더 많음) → 형식 오류로 줄 전체 버림
    size_t i = 0;
    while (true)
    {
        if (*text == ',' || *text == '\0')
        {
            out[i] = NAN;  // 빈칸 = 유지 (바퀴는 0으로, 팔·그리퍼는 명령 생략)
        }
        else
        {
            char *end = nullptr;
            out[i] = strtof(text, &end);
            if (end == text || !isfinite(out[i]))
                return false;
            text = end;
        }
        ++i;
        if (*text == '\0')
            break;
        if (*text != ',')
            return false;
        ++text;
        if (i == count)
            return false;  // 필드가 count개를 넘음
    }
    for (; i < count; ++i)
        out[i] = NAN;  // 생략된 뒷 필드 = 유지
    return true;
}

// T 명령 — 토크 온/오프. 필드 8개 = C와 같은 순서(바퀴L, 바퀴R, joint1~5, 그리퍼).
//   T,0                     → 전체 해제
//   T,1                     → 전체 켜기 (지금 자세를 목표로)
//   T,1,0,,,,,,             → 왼바퀴만 켜고 오른바퀴 끔, 나머지 유지
//   T,,0,,,,,,             → joint1만 끄기 (빈칸 = 지금 상태 유지)
// 켤 때는 그 관절의 현재 위치를 목표로 잡아 튀지 않게 한다.
constexpr size_t TORQUE_COUNT = 2 + ARM_COUNT + 1;
bool torque_state[TORQUE_COUNT] = {true, true, true, true, true, true, true, true};

uint8_t torque_id(size_t i)
{
    if (i == 0)
        return WHEEL_LEFT_ID;
    if (i == 1)
        return WHEEL_RIGHT_ID;
    if (i < 2 + ARM_COUNT)
        return ARM_IDS[i - 2];
    return GRIPPER_ID;
}

void set_torque_one(size_t i, bool on)
{
    const uint8_t id = torque_id(i);
    if (on)
    {
        if (i < 2)
            dxl.setGoalVelocity(id, 0);  // 바퀴는 속도 모드 — 0으로
        else
            dxl.setGoalPosition(id, dxl.getPresentPosition(id, UNIT_DEGREE), UNIT_DEGREE);
        dxl.torqueOn(id);
    }
    else
    {
        dxl.torqueOff(id);
    }
    torque_state[i] = on;
}

bool any_torque_on()
{
    for (size_t i = 0; i < TORQUE_COUNT; ++i)
        if (torque_state[i])
            return true;
    return false;
}

void apply_torque_line()
{
    if (!motors_ready || strncmp(line, "T,", 2) != 0)
        return;
    const bool whole = (strchr(line + 2, ',') == nullptr);  // T,0 / T,1 = 전체
    float v[TORQUE_COUNT];
    if (!parse_fields(line + 2, v, TORQUE_COUNT))
        return;
    if (whole)
    {
        if (isnan(v[0]))
            return;
        const bool on = v[0] != 0.0f;
        for (size_t i = 0; i < TORQUE_COUNT; ++i)
            set_torque_one(i, on);
    }
    else
    {
        for (size_t i = 0; i < TORQUE_COUNT; ++i)
            if (!isnan(v[i]))
                set_torque_one(i, v[i] != 0.0f);
    }
    // 하나라도 토크가 켜져 있으면 watchdog 유지, 전부 꺼졌으면 watchdog 정지
    watchdog_tripped = !any_torque_on();
    last_command_ms = millis();
}

void apply_line()
{
    if (!motors_ready)
        return;
    if (strncmp(line, "T,", 2) == 0)
    {
        apply_torque_line();
        return;
    }
    if (strncmp(line, "C,", 2) != 0)
        return;
    float v[COMMAND_FIELDS];
    if (!parse_fields(line + 2, v, COMMAND_FIELDS))
        return;
    if (isnan(v[0]))
        dxl.setGoalVelocity(WHEEL_LEFT_ID, 0);  // 빈칸 = 정지
    else if (torque_state[0])
        dxl.setGoalVelocity(WHEEL_LEFT_ID, rad_s_to_rpm(v[0] * WHEEL_LEFT_DIR), UNIT_RPM);
    if (isnan(v[1]))
        dxl.setGoalVelocity(WHEEL_RIGHT_ID, 0);  // 빈칸 = 정지
    else if (torque_state[1])
        dxl.setGoalVelocity(WHEEL_RIGHT_ID, rad_s_to_rpm(v[1] * WHEEL_RIGHT_DIR), UNIT_RPM);
    for (size_t i = 0; i < ARM_COUNT; ++i)
    {
        if (!isnan(v[2 + i]) && torque_state[2 + i])
            dxl.setGoalPosition(ARM_IDS[i], joint_rad_to_deg(v[2 + i], ARM_DIR[i]), UNIT_DEGREE);
    }
    if (!isnan(v[2 + ARM_COUNT]) && torque_state[2 + ARM_COUNT])
        dxl.setGoalPosition(GRIPPER_ID, joint_rad_to_deg(v[2 + ARM_COUNT] * GRIPPER_RAD_PER_M, 1.0f), UNIT_DEGREE);
    last_command_ms = millis();
    if (any_torque_on())
        watchdog_tripped = false;
}

void check_command_timeout()
{
    if (!motors_ready || watchdog_tripped || millis() - last_command_ms < COMMAND_TIMEOUT_MS)
        return;
    hold_present_pose();
    watchdog_tripped = true;
}

void print_value(float value)
{
    Serial.print(',');
    Serial.print(value, 5);
}

void send_state()
{
    Serial.print('S');
    const uint8_t wheels[] = {WHEEL_LEFT_ID, WHEEL_RIGHT_ID};
    const float wheel_dir[] = {WHEEL_LEFT_DIR, WHEEL_RIGHT_DIR};
    for (size_t i = 0; i < 2; ++i)
    {
        print_value(wheel_dir[i] * dxl.getPresentPosition(wheels[i], UNIT_RAW) * 2.0f * PI / 4096.0f);
        print_value(wheel_dir[i] * rpm_to_rad_s(dxl.getPresentVelocity(wheels[i], UNIT_RPM)));
    }
    for (size_t i = 0; i < ARM_COUNT; ++i)
    {
        print_value(deg_to_joint_rad(dxl.getPresentPosition(ARM_IDS[i], UNIT_DEGREE), ARM_DIR[i]));
        print_value(ARM_DIR[i] * rpm_to_rad_s(dxl.getPresentVelocity(ARM_IDS[i], UNIT_RPM)));
        print_value(dxl.getPresentCurrent(ARM_IDS[i], UNIT_MILLI_AMPERE) / 1000.0f);
    }
    print_value(deg_to_joint_rad(dxl.getPresentPosition(GRIPPER_ID, UNIT_DEGREE), 1.0f) / GRIPPER_RAD_PER_M);
    print_value(rpm_to_rad_s(dxl.getPresentVelocity(GRIPPER_ID, UNIT_RPM)) / GRIPPER_RAD_PER_M);
    print_value(dxl.getPresentCurrent(GRIPPER_ID, UNIT_MILLI_AMPERE) / 1000.0f);

    imu.update();
    print_value(imu.quat[1]);  // cIMU quat = w, x, y, z
    print_value(imu.quat[2]);
    print_value(imu.quat[3]);
    print_value(imu.quat[0]);
    for (size_t i = 0; i < 3; ++i)
        print_value(imu.SEN.gyroADC[i] * GYRO_FACTOR);
    for (size_t i = 0; i < 3; ++i)
        print_value(imu.SEN.accADC[i] * ACCEL_FACTOR);
    Serial.println();
}

void setup()
{
    Serial.begin(HOST_BAUD);
    imu.begin();
    dxl.begin(DXL_BAUD);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    bool ok = setup_motor(WHEEL_LEFT_ID, OP_VELOCITY) && setup_motor(WHEEL_RIGHT_ID, OP_VELOCITY);
    for (size_t i = 0; i < ARM_COUNT; ++i)
        ok = setup_motor(ARM_IDS[i], OP_POSITION) && ok;
    ok = setup_motor(GRIPPER_ID, OP_CURRENT_BASED_POSITION) && ok;  // 퍽을 쥐면 전류로 멈춘다
    motors_ready = ok;  // 하나라도 응답이 없으면 명령을 받지 않는다 (상태는 계속 보냄 — SBC가 원인 파악)
}

void loop()
{
    check_command_timeout();
    while (Serial.available() > 0)
    {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\n')
        {
            line[line_length] = '\0';
            apply_line();
            line_length = 0;
        }
        else if (ch != '\r' && line_length < LINE_CAPACITY - 1)
        {
            line[line_length++] = ch;
        }
        else if (line_length >= LINE_CAPACITY - 1)
        {
            line_length = 0;  // 너무 긴 줄은 버림
        }
    }
    if (millis() - last_state_ms >= STATE_PERIOD_MS)
    {
        last_state_ms = millis();
        send_state();
    }
}
