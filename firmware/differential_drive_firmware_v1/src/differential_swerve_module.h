#ifndef DIFFERENTIAL_SWERVE_MODULE_H
#define DIFFERENTIAL_SWERVE_MODULE_H

#include <cmath> // In C++, it's better to include standard headers here

class DifferentialSwerveModule {
public:
    /**
     * @brief Constructor ของโมดูล
     * @param ticks_per_rev จำนวน Ticks ต่อ 1 รอบของมอเตอร์
     * @param motor_gear_ratio อัตราทดเกียร์จากมอเตอร์ไปยังเกียร์ที่ขับให้โมดูลหมุน
     * @param module_track_width_mm ระยะห่างระหว่างจุดสัมผัสของเกียร์ขับซ้าย-ขวา (W) หน่วยเป็น mm
     * @param drive_gear_diameter_mm เส้นผ่านศูนย์กลางของเกียร์ที่ขับให้โมดูลหมุน หน่วยเป็น mm
     */
    DifferentialSwerveModule(double ticks_per_rev, double motor_gear_ratio, double module_track_width_mm, double drive_gear_diameter_mm);

    /**
     * @brief อัปเดตมุมของโมดูลโดยรับค่า Ticks ปัจจุบันของ Encoder ทั้งสอง
     * @param current_total_ticks_L ค่า Ticks ทั้งหมดที่อ่านได้จาก Encoder ซ้าย
     * @param current_total_ticks_R ค่า Ticks ทั้งหมดที่อ่านได้จาก Encoder ขวา
     * @return มุมปัจจุบันของโมดูลในหน่วยองศา (ในช่วง -180 ถึง 180)
     */
    double update_angle(long long current_total_ticks_L, long long current_total_ticks_R);

    /**
     * @brief ทำการ Reset หรือ 'Homing' มุมของโมดูลให้เป็น 0
     */
    void zero_angle();
    
    /**
     * @brief ดึงค่ามุมปัจจุบัน
     * @return มุมปัจจุบัน (องศา)
     */
    double get_current_angle() const;

    /**
     * @brief ทำให้มุมอยู่ในช่วง -180 ถึง 180 องศา
     * @param angle_deg มุมที่ต้องการจัดระเบียบ
     * @return มุมที่จัดระเบียบแล้ว
     */
    double normalize_angle(double angle_deg);

private:
    // ค่าคงที่ทางกายภาพ
    const double TICKS_PER_MOTOR_REV;
    const double MOTOR_GEAR_RATIO;
    const double MODULE_TRACK_WIDTH_MM;
    const double DRIVE_GEAR_DIAMETER_MM;

    // ค่าคงที่ที่คำนวณได้
    double TICKS_PER_360_DEG_ROTATION;

    // ตัวแปรสถานะของโมดูล
    double current_angle_deg;
    long long last_total_ticks_L;
    long long last_total_ticks_R;
};

#endif // DIFFERENTIAL_SWERVE_MODULE_HPP