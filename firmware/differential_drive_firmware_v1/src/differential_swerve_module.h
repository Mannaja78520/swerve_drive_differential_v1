#ifndef DIFFERENTIAL_SWERVE_MODULE_H
#define DIFFERENTIAL_SWERVE_MODULE_H

class DifferentialSwerveModule {
public:
    /**
     * @brief Constructor ของโมดูล (เวอร์ชันใหม่ ใช้จำนวนซี่เฟือง)
     * @param ticks_per_rev จำนวน Ticks ต่อ 1 รอบของมอเตอร์
     * @param motor_gear_ratio อัตราทดเกียร์จากมอเตอร์ไปยังเฟืองขับ
     * @param stationary_gear_teeth จำนวนซี่ของเฟืองวงแหวนที่โมดูลหมุนรอบ
     * @param drive_gear_teeth จำนวนซี่ของเฟืองขับที่ติดกับมอเตอร์
     */
    DifferentialSwerveModule(float ticks_per_rev, float motor_gear_ratio, int stationary_gear_teeth, int drive_gear_teeth);

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

    /**
     * @brief ทำให้มุมอยู่ในช่วง -180 ถึง 180 องศา
     * @param angle_deg มุมที่ต้องการจัดระเบียบ
     * @return มุมที่จัดระเบียบแล้ว
     */
    float normalize_angle(float angle_deg);

private:
    // ค่าคงที่ทางกายภาพ
    const float TICKS_PER_MOTOR_REV;
    const float MOTOR_GEAR_RATIO;
    // ไม่จำเป็นต้องเก็บค่าซี่เฟืองไว้ เพราะใช้คำนวณครั้งเดียวใน Constructor

    // ค่าคงที่ที่คำนวณได้
    float TICKS_PER_360_DEG_ROTATION;

    // ตัวแปรสถานะของโมดูล
    float current_angle_deg;
    long long last_total_ticks_L;
    long long last_total_ticks_R;
};

#endif // DIFFERENTIAL_SWERVE_MODULE_H