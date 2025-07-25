#include "differential_swerve_module.h"
#include <cmath> // For M_PI and fmod

// Constructor (เวอร์ชันใหม่ ใช้จำนวนซี่เฟือง)
DifferentialSwerveModule::DifferentialSwerveModule(
    float ticks_per_rev,
    float motor_gear_ratio,
    int stationary_gear_teeth, // << จำนวนซี่ของเฟืองวงแหวนที่โมดูลหมุนรอบ
    int drive_gear_teeth       // << จำนวนซี่ของเฟืองขับที่ติดกับมอเตอร์
)
    : TICKS_PER_MOTOR_REV(ticks_per_rev),
      MOTOR_GEAR_RATIO(motor_gear_ratio),
      current_angle_deg(0.0),
      last_total_ticks_L(0),
      last_total_ticks_R(0)
{
    // --- จุดที่แก้ไข ---
    // คำนวณอัตราส่วนการหมุนโดยใช้จำนวนซี่เฟืองโดยตรง
    float gear_revs_for_360_turn = static_cast<float>(stationary_gear_teeth) / drive_gear_teeth;

    // การคำนวณส่วนที่เหลือยังคงเหมือนเดิม
    float motor_revs_for_360_turn = gear_revs_for_360_turn * this->MOTOR_GEAR_RATIO;
    this->TICKS_PER_360_DEG_ROTATION = gear_revs_for_360_turn * this->TICKS_PER_MOTOR_REV;
}

// ฟังก์ชันหลักสำหรับอัปเดตมุม (ไม่ต้องแก้ไข)
float DifferentialSwerveModule::update_angle(long long current_total_ticks_L, long long current_total_ticks_R) {
    // คำนวณหาการเปลี่ยนแปลงของ Ticks จากรอบที่แล้ว
    long long delta_ticks_L = current_total_ticks_L - this->last_total_ticks_L;
    long long delta_ticks_R = current_total_ticks_R - this->last_total_ticks_R;

    // ผลต่างของ Ticks ที่ส่งผลต่อการเลี้ยว
    long long steering_tick_diff = (delta_ticks_L - delta_ticks_R) / 2.0;

    // คำนวณองศาที่เปลี่ยนแปลงไป
    float delta_angle = (static_cast<float>(steering_tick_diff) / this->TICKS_PER_360_DEG_ROTATION) * 360.0;

    // อัปเดตมุมปัจจุบัน
    this->current_angle_deg += delta_angle;

    // อัปเดตค่า Ticks ล่าสุดสำหรับใช้ในรอบถัดไป
    this->last_total_ticks_L = current_total_ticks_L;
    this->last_total_ticks_R = current_total_ticks_R;

    // ทำให้มุมอยู่ในช่วงที่ต้องการแล้วส่งค่ากลับ
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