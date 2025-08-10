#include <Arduino.h>

#include <sensor_msgs/msg/imu.h>
#include <imu_bno055.h>

#include <config.h>

IMU_BNO055 bno055;
void setup() {
    Serial.begin(115200);
    pinMode(IMU_RST, OUTPUT);
    pinMode(IMU_INT, INPUT);

    digitalWrite(IMU_RST, LOW);
    delay(10);
    digitalWrite(IMU_RST, HIGH);
    delay(50);
    
    while (!Serial) {
        delay(10); // wait for serial port to connect. Needed for native USB
    }

    Serial.println("IMU Test Start");

    if (!bno055.init()) {
        Serial.println("Failed to initialize IMU");
        return;
    }

}

void loop (){
    
    sensor_msgs__msg__Imu imu_msg;
    bno055.getIMUData(imu_msg);
    
    Serial.print("Orientation: ");
    Serial.print(imu_msg.orientation.x);
    Serial.print(", ");
    Serial.print(imu_msg.orientation.y);
    Serial.print(", ");
    Serial.print(imu_msg.orientation.z);
    Serial.print(", ");
    Serial.println(imu_msg.orientation.w);
    Serial.print("Angular Velocity: ");
    Serial.print(imu_msg.angular_velocity.x);
    Serial.print(", ");
    Serial.print(imu_msg.angular_velocity.y);
    Serial.print(", ");
    Serial.println(imu_msg.angular_velocity.z);
    Serial.print("Linear Acceleration: ");
    Serial.print(imu_msg.linear_acceleration.x);
    Serial.print(", ");
    Serial.print(imu_msg.linear_acceleration.y);
    Serial.print(", ");
    Serial.println(imu_msg.linear_acceleration.z);
    Serial.println("-----------------------");
}