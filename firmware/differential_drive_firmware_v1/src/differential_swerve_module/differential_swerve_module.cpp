#include "differential_swerve_module.h"
#include <cmath> // For M_PI and fmod

// Constructor (เวอร์ชันใหม่ ใช้จำนวนซี่เฟือง)
DifferentialSwerveModule::DifferentialSwerveModule(
    float ticks_per_rev,
    float motor_gear_ratio,
    int stationary_gear_teeth,
    int drive_gear_teeth,
    float wheel_distance_L,
    int module_index)
    : TICKS_PER_MOTOR_REV(ticks_per_rev),
      MOTOR_GEAR_RATIO(motor_gear_ratio),
      module_index(module_index),
      current_angle_deg(0.0),
      last_total_ticks_L(0),
      last_total_ticks_R(0)
{
    // อัตราทดจากมอเตอร์ -> เฟือง bevel ที่ขับโมดูลหมุน (steering gear train)
    float steering_motor_to_drivegear_ratio = this->MOTOR_GEAR_RATIO; // เฉพาะชุดเฟืองจากมอเตอร์ถึง drive gear ของ ring gear

    // อัตราทดระหว่าง stationary ring gear กับ drive gear
    float ring_to_drivegear_ratio = static_cast<float>(stationary_gear_teeth) / static_cast<float>(drive_gear_teeth);

    // จำนวน ticks ที่ทำให้โมดูลหมุนครบ 360°
    this->TICKS_PER_360_DEG_ROTATION = ring_to_drivegear_ratio * this->TICKS_PER_MOTOR_REV;

    // กำหนดมุมเริ่มต้นตาม module_index
    switch(module_index) {
        case 0: // ล้อหน้า
            this->wheel_angle_offset = this->front_wheel_angle; // หรือ 90.0f ตามที่คุณต้องการ
            this->wheel_positions = {{wheel_distance_L, 0}};
            break;
        case 1: // ล้อหลังซ้าย
            this->wheel_angle_offset = this->rear_left_wheel_angle; // 120 องศาจากแกน X
            this->wheel_positions = {{-std::sqrt(3) * wheel_distance_L / 2.0f, wheel_distance_L / 2.0f}};
            break;
        case 2: // ล้อหลังขวา
            this->wheel_angle_offset = this->rear_right_wheel_angle; // -120 องศาจากแกน X
            this->wheel_positions = {{-std::sqrt(3) * wheel_distance_L / 2.0f, -wheel_distance_L / 2.0f}};
            break;
        default:
            this->wheel_angle_offset = 0.0f;
            this->wheel_positions = {{0, 0}};
    }
    this->current_angle_deg = this->wheel_angle_offset; // เริ่มต้นมุมเป็นมุมเริ่มต้นของโมดูลนี้
}

float DifferentialSwerveModule::get_wheel_angle() const {
    return this->wheel_angle_offset;
}
// ฟังก์ชันหลักสำหรับอัปเดตมุม (ไม่ต้องแก้ไข)
float DifferentialSwerveModule::update_angle(long long current_total_ticks_L, long long current_total_ticks_R) {
    long long delta_ticks_L = current_total_ticks_L - this->last_total_ticks_L;
    long long delta_ticks_R = current_total_ticks_R - this->last_total_ticks_R;

    long long steering_tick_diff = (static_cast<float>(delta_ticks_L) - static_cast<float>(delta_ticks_R));
    float delta_angle = ((static_cast<float>(steering_tick_diff) * 0.5) / this->TICKS_PER_360_DEG_ROTATION) * 360.0;
    // float delta_angle = (static_cast<float>(delta_ticks_L - delta_ticks_R) / 
    //                     (2.0f * this->TICKS_PER_360_DEG_ROTATION)) * 360.0f;

    this->current_angle_deg = normalize_angle(this->current_angle_deg + delta_angle);
    this->last_total_ticks_L = current_total_ticks_L;
    this->last_total_ticks_R = current_total_ticks_R;

    return this->current_angle_deg;
}

// ฟังก์ชันรีเซ็ตมุม (ไม่ต้องแก้ไข)
void DifferentialSwerveModule::zero_angle() {
    this->last_total_ticks_L = 0;
    this->last_total_ticks_R = 0;
    this->current_angle_deg = this->wheel_angle_offset; // รีเซ็ตมุมเป็นมุมเริ่มต้นของโมดูลนี้
}

// ฟังก์ชันดึงค่ามุม (ไม่ต้องแก้ไข)
float DifferentialSwerveModule::get_current_angle() const {
    return this->current_angle_deg;
}

// ฟังก์ชันจัดระเบียบมุม (ไม่ต้องแก้ไข)
float DifferentialSwerveModule::normalize_angle(float angle_deg) const {
    float angle = fmod(angle_deg, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}


// ฟังก์ชันคำนวณความเร็วและมุมของล้อ (ใหม่)

// std::vector<std::pair<float, float>> DifferentialSwerveModule::kinematics(float Vx, float Vy, float omega) const {
//     std::vector<std::pair<float, float>> motor_speeds;
    
//     // แต่ละโมดูลมีเพียง 1 ตำแหน่งล้อ (ตาม module_index)
//     const auto& pos = this->wheel_positions[0];
//     float x = pos.first;
//     float y = pos.second;

//     // คำนวณความเร็วสัมพัทธ์
//     float vx = Vx - omega * y;
//     float vy = Vy + omega * x;

//     float speed = std::hypot(vx, vy);
//     float angle_rad = (speed < 0.001f) ? 0.0f : std::atan2(vy, vx);
//     float angle_deg = this->normalize_angle(angle_rad * 180.0f / M_PI);

//     motor_speeds.push_back({speed, angle_deg});
    
//     return motor_speeds;
// }


// std::vector<std::pair<float, float>> DifferentialSwerveModule::kinematics(float Vx, float Vy, float omega) const {
//     std::vector<std::pair<float, float>> wheel_commands;
    
//     // คำนวณความเร็วสัมพัทธ์ของโมดูล (ในกรอบอ้างอิงโลก)
//     float module_vx = Vx - omega * wheel_positions[0].second;  // Vx - ω*y
//     float module_vy = Vy + omega * wheel_positions[0].first;  // Vy + ω*x
    
//     // แปลงเป็นความเร็วในกรอบอ้างอิงของโมดูล (หมุนตามมุมของโมดูล)
//     float current_angle_rad = (current_angle_deg - wheel_angle_offset) * M_PI / 180.0f;
//     float rotated_vx = module_vx * cos(current_angle_rad) + module_vy * sin(current_angle_rad);
//     float rotated_vy = -module_vx * sin(current_angle_rad) + module_vy * cos(current_angle_rad);
    
//     // คำนวณความเร็วและทิศทางของล้อ
//     float speed = sqrt(rotated_vx * rotated_vx + rotated_vy * rotated_vy);
//     float angle_rad = (speed < 0.001f) ? 0.0f : atan2(rotated_vy, rotated_vx);
    
//     // เพิ่มคำสั่งความเร็วและมุมล้อ (แปลงเป็นองศา)
//     wheel_commands.push_back({speed, angle_rad * 180.0f / M_PI});
    
//     return wheel_commands;
// }


std::vector<std::pair<float, float>> DifferentialSwerveModule::kinematics(float Vx, float Vy, float omega) const {
    std::vector<std::pair<float, float>> motor_speeds;
    
    const auto& pos = this->wheel_positions[0];
    float x = pos.first;
    float y = pos.second;

    // Calculate velocity components for this module
    float module_vx = Vx + (omega * y * omega_gain);
    float module_vy = Vy - (omega * x * omega_gain);

    // Calculate desired speed (magnitude)
    float desired_speed = sqrt(module_vx*module_vx + module_vy*module_vy);

    // Calculate desired wheel angle (global frame)
    float desired_angle_deg = atan2(module_vy, module_vx) * 180.0f / M_PI;
    desired_angle_deg = optimizeAngle(desired_angle_deg, current_angle_deg, desired_speed);
    

    // Convert to module's local frame
    float angle_diff = desired_angle_deg - this->current_angle_deg;
    angle_diff = normalize_angle(angle_diff);

    // The actual wheel speed should combine translation and rotation
    motor_speeds.push_back({desired_speed, desired_angle_deg});
    
    return motor_speeds;
}

float DifferentialSwerveModule::optimizeAngle(float desired_angle, float current_angle, float &wheel_speed) const{
    float delta = normalize_angle(desired_angle - current_angle);
    if (std::fabs(delta) > 90.0f) {
        desired_angle = normalize_angle(desired_angle + 180.0f);
        wheel_speed = -wheel_speed;
    }
    return desired_angle;
}