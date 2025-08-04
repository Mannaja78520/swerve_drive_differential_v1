#include "differential_swerve_module.h"
#include <cmath> // For M_PI and fmod

// Constructor (เวอร์ชันใหม่ ใช้จำนวนซี่เฟือง)
DifferentialSwerveModule::DifferentialSwerveModule(
    float ticks_per_rev,
    float motor_gear_ratio,
    int stationary_gear_teeth, // << จำนวนซี่ของเฟืองวงแหวนที่โมดูลหมุนรอบ
    int drive_gear_teeth,      // << จำนวนซี่ของเฟืองขับที่ติดกับมอเตอร์
    float wheel_distance_L     // << ระยะห่างจากจุดหมุนกลางไปยังล้อ
)
    : TICKS_PER_MOTOR_REV(ticks_per_rev),
      MOTOR_GEAR_RATIO(motor_gear_ratio),
      current_angle_deg(0.0),
      last_total_ticks_L(0),
      last_total_ticks_R(0)
{
    float gear_revs_for_360_turn = static_cast<float>(stationary_gear_teeth) / drive_gear_teeth;
    this->TICKS_PER_360_DEG_ROTATION = gear_revs_for_360_turn * this->MOTOR_GEAR_RATIO * this->TICKS_PER_MOTOR_REV;

    // ตั้งค่าตำแหน่งล้อ
    this->wheel_positions = {
        {wheel_distance_L, 0},                                      // ล้อหน้า
        {-wheel_distance_L / 2, std::sqrt(3) * wheel_distance_L / 2}, // ล้อหลังซ้าย
        {-wheel_distance_L / 2, -std::sqrt(3) * wheel_distance_L / 2} // ล้อหลังขวา
    };
}

// ฟังก์ชันหลักสำหรับอัปเดตมุม (ไม่ต้องแก้ไข)
float DifferentialSwerveModule::update_angle(long long current_total_ticks_L, long long current_total_ticks_R) {
    long long delta_ticks_L = current_total_ticks_L - this->last_total_ticks_L;
    long long delta_ticks_R = current_total_ticks_R - this->last_total_ticks_R;

    long long steering_tick_diff = (delta_ticks_L - delta_ticks_R) / 2.0;
    float delta_angle = (static_cast<float>(steering_tick_diff) / this->TICKS_PER_360_DEG_ROTATION) * 360.0;

    this->current_angle_deg += delta_angle;
    this->last_total_ticks_L = current_total_ticks_L;
    this->last_total_ticks_R = current_total_ticks_R;

    this->current_angle_deg = normalize_angle(this->current_angle_deg);
    return this->current_angle_deg;
}

// ฟังก์ชันรีเซ็ตมุม (ไม่ต้องแก้ไข)
void DifferentialSwerveModule::zero_angle() {
    this->current_angle_deg = 0.0;
}

// ฟังก์ชันดึงค่ามุม (ไม่ต้องแก้ไข)
float DifferentialSwerveModule::get_current_angle() const {
    return this->current_angle_deg;
}

// ฟังก์ชันจัดระเบียบมุม (ไม่ต้องแก้ไข)
float DifferentialSwerveModule::normalize_angle(float angle_deg) {
    float angle = fmod(angle_deg, 360.0);
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}

// ฟังก์ชันคำนวณความเร็วและมุมของล้อ (ใหม่)
std::vector<std::pair<float, float>> DifferentialSwerveModule::kinematics(float Vx, float Vy, float omega) const {
    std::vector<std::pair<float, float>> motor_speeds;

    for (const auto& pos : this->wheel_positions) {
        float x = pos.first;
        float y = pos.second;

        float vix = Vx - omega * y;
        float viy = Vy + omega * x;

        float speed = std::hypot(vix, viy);
        float angle = (vix == 0 && viy == 0) ? 0.0f : std::atan2(viy, vix);

        motor_speeds.push_back({speed, angle});
    }

    return motor_speeds;
}
