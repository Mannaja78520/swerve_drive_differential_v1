#include "differential_swerve_module.h"

// Constructor
DifferentialSwerveModule::DifferentialSwerveModule(double ticks_per_rev, double motor_gear_ratio, double module_track_width_mm, double drive_gear_diameter_mm)
    : TICKS_PER_MOTOR_REV(ticks_per_rev),
      MOTOR_GEAR_RATIO(motor_gear_ratio),
      MODULE_TRACK_WIDTH_MM(module_track_width_mm),
      DRIVE_GEAR_DIAMETER_MM(drive_gear_diameter_mm),
      current_angle_deg(0.0),
      last_total_ticks_L(0),
      last_total_ticks_R(0) 
{
    // คำนวณค่าคงที่สำหรับการแปลง Ticks เป็นองศา
    double drive_gear_circumference_mm = this->DRIVE_GEAR_DIAMETER_MM * M_PI;
    double module_rotation_circumference_mm = this->MODULE_TRACK_WIDTH_MM * M_PI;
    double gear_revs_for_360_turn = module_rotation_circumference_mm / drive_gear_circumference_mm;
    double motor_revs_for_360_turn = gear_revs_for_360_turn * this->MOTOR_GEAR_RATIO;
    this->TICKS_PER_360_DEG_ROTATION = motor_revs_for_360_turn * this->TICKS_PER_MOTOR_REV;
}

// ฟังก์ชันหลักสำหรับอัปเดตมุม
double DifferentialSwerveModule::update_angle(long long current_total_ticks_L, long long current_total_ticks_R) {
    // คำนวณหาการเปลี่ยนแปลงของ Ticks จากรอบที่แล้ว
    long long delta_ticks_L = current_total_ticks_L - this->last_total_ticks_L;
    long long delta_ticks_R = current_total_ticks_R - this->last_total_ticks_R;

    // ผลต่างของ Ticks ที่ส่งผลต่อการเลี้ยว
    long long steering_tick_diff = delta_ticks_L - delta_ticks_R;

    // คำนวณองศาที่เปลี่ยนแปลงไป
    double delta_angle = (static_cast<double>(steering_tick_diff) / this->TICKS_PER_360_DEG_ROTATION) * 360.0;

    // อัปเดตมุมปัจจุบัน
    this->current_angle_deg += delta_angle;
    
    // อัปเดตค่า Ticks ล่าสุดสำหรับใช้ในรอบถัดไป
    this->last_total_ticks_L = current_total_ticks_L;
    this->last_total_ticks_R = current_total_ticks_R;

    // ทำให้มุมอยู่ในช่วงที่ต้องการแล้วส่งค่ากลับ
    this->current_angle_deg = normalize_angle(this->current_angle_deg);
    return this->current_angle_deg;
}

// ฟังก์ชันรีเซ็ตมุม
void DifferentialSwerveModule::zero_angle() {
    this->current_angle_deg = 0.0;
}

// ฟังก์ชันดึงค่ามุม
double DifferentialSwerveModule::get_current_angle() const {
    return this->current_angle_deg;
}

// ฟังก์ชันจัดระเบียบมุม
double DifferentialSwerveModule::normalize_angle(double angle_deg) {
    double angle = fmod(angle_deg, 360.0);
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}
