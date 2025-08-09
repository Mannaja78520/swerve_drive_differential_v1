#ifndef DIFFERENTIAL_SWERVE_MODULE_H
#define DIFFERENTIAL_SWERVE_MODULE_H

#include <vector>
#include <utility>

class DifferentialSwerveModule {
public:
    const float front_wheel_angle = 90.0f; // มุมของล้อหน้า
    const float rear_left_wheel_angle = 30.0f;
    const float rear_right_wheel_angle = 150.0f; // มุมของล้อหลังขวา

    /**
     * @brief Constructor ของโมดูล (เวอร์ชันใหม่ ใช้จำนวนซี่เฟือง)
     * @param ticks_per_rev จำนวน Ticks ต่อ 1 รอบของมอเตอร์
     * @param motor_gear_ratio อัตราทดเกียร์จากมอเตอร์ไปยังเฟืองขับ
     * @param stationary_gear_teeth จำนวนซี่ของเฟืองวงแหวนที่โมดูลหมุนรอบ
     * @param drive_gear_teeth จำนวนซี่ของเฟืองขับที่ติดกับมอเตอร์
     * @param wheel_distance_L ระยะจากจุดหมุนกลางไปยังแต่ละล้อ
     */
    DifferentialSwerveModule(float ticks_per_rev, float motor_gear_ratio,
                             int stationary_gear_teeth, int drive_gear_teeth,
                             float wheel_distance_L, int module_index = 0);

    /**
     * @brief อัปเดตมุมของโมดูลโดยรับค่า Ticks ปัจจุบันของ Encoder ทั้งสอง
     * @param current_total_ticks_L ค่า Ticks ทั้งหมดที่อ่านได้จาก Encoder ซ้าย
     * @param current_total_ticks_R ค่า Ticks ทั้งหมดที่อ่านได้จาก Encoder ขวา
     * @return มุมปัจจุบันของโมดูลในหน่วยองศา (ในช่วง -180 ถึง 180)
     */
    float update_angle(long long current_total_ticks_L, long long current_total_ticks_R);

    /**
     * @brief ทำการ Reset หรือ 'Homing' มุมของโมดูลให้เป็น 0
     */
    void zero_angle();
    
    /**
     * @brief ดึงค่ามุมปัจจุบัน
     * @return มุมปัจจุบัน (องศา)
     */
    float get_current_angle() const;

    float get_wheel_angle() const;

    /**
     * @brief ทำให้มุมอยู่ในช่วง -180 ถึง 180 องศา
     * @param angle_deg มุมที่ต้องการจัดระเบียบ
     * @return มุมที่จัดระเบียบแล้ว
     */
    float normalize_angle(float angle_deg) const;

    /**
     * @brief คำนวณความเร็วและมุมของล้อทั้ง 3 จาก Vx, Vy, omega
     * @param Vx ความเร็วแกน X (เดินหน้า/ถอยหลัง)
     * @param Vy ความเร็วแกน Y (เลี้ยวข้าง)
     * @param omega ความเร็วเชิงมุม (หมุนตัว)
     * @return รายการของความเร็วและมุม (rad) ของล้อแต่ละล้อ
     */
    std::vector<std::pair<float, float>> kinematics(float Vx, float Vy, float omega) const;

    /**
     * @brief รีเซ็ตมุมของโมดูลเป็น 0
     */
    void set_zero();

private:
    // ค่าคงที่ทางกายภาพ
    const float TICKS_PER_MOTOR_REV;
    const float MOTOR_GEAR_RATIO;

    // ค่าคงที่ที่คำนวณได้
    float TICKS_PER_360_DEG_ROTATION;

    // ตัวแปรสถานะของโมดูล
    float current_angle_deg;
    long long last_total_ticks_L;
    long long last_total_ticks_R;
    int module_index; // เพิ่มตัวแปรเก็บ index
    float wheel_angle_offset; // มุมเริ่มต้นของโมดูลนี้

    // ตำแหน่งของล้อแต่ละล้อ (X, Y)
    std::vector<std::pair<float, float>> wheel_positions;
};

#endif // DIFFERENTIAL_SWERVE_MODULE_H

