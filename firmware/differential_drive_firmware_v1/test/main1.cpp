#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <utility>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <Utilize.h>
#include <differential_swerve_module/differential_swerve_module.h>
#include <esp32_Encoder.h>    
#include <ESP32Servo.h>

// Macro definitions and global variables
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { } }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)

// Publishers and subscribers
rcl_publisher_t debug_move_wheel_motor_publisher;
rcl_publisher_t debug_move_wheel_encoder_publisher;
rcl_publisher_t debug_hall_sensor1_publisher;
rcl_publisher_t debug_hall_sensor2_publisher;

rcl_subscription_t move_wheel_motor_subscriber;
rcl_subscription_t movement_mode_subscriber;
rcl_subscription_t cmd_vel_subscriber;

// Messages
std_msgs__msg__String movement_mode_msg;
std_msgs__msg__Bool hall_sensor1_msg;
std_msgs__msg__Bool hall_sensor2_msg;
geometry_msgs__msg__Twist debug_wheel_motor_msg;
geometry_msgs__msg__Twist debug_wheel_encoder_msg;
geometry_msgs__msg__Twist moveMotor_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// ROS2 entities
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

// Encoder and motor variables
long long ticks_L_front = 0;
long long ticks_R_front = 0;
long long ticks_L_rear_left = 0;
long long ticks_R_rear_left = 0;

float angle_front = 0.0;
float angle_rear_left = 0.0;
float rpm_front_L = 0;
float rpm_front_R = 0;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;
static unsigned long last_pub = 0;
static int disconnect_count = 0;

bool hall_sensor1;
bool hall_sensor2;

float V_x = 0.0;
float V_y = 0.0;
float Omega_z = 0.0;

String movement_mode = "mps"; 
float motor1_target = 0.0;
float motor2_target = 0.0;
float motor3_target = 0.0;

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// PID controllers
PIDF front_L_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
PIDF front_R_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
PIDF rear_left_L_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
PIDF rear_left_R_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);

// Motors and encoders
Controller motor1(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BRAKE, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Controller motor3(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_BRAKE, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_BRAKE, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

esp32_Encoder Encoder1(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV, MOTOR1_ENCODER_RATIO);
esp32_Encoder Encoder2(MOTOR2_ENCODER_PIN_A, MOTOR2_ENCODER_PIN_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV, MOTOR2_ENCODER_RATIO);
esp32_Encoder Encoder3(MOTOR3_ENCODER_PIN_A, MOTOR3_ENCODER_PIN_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV, MOTOR3_ENCODER_RATIO);
esp32_Encoder Encoder4(MOTOR4_ENCODER_PIN_A, MOTOR4_ENCODER_PIN_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV, MOTOR4_ENCODER_RATIO);

// Wheel angle PID controllers
PIDF Angle_Wheel1_pidf(PWM_Min, PWM_Max, Wheel_Spin_Motor_KP, Wheel_Spin_Motor_KI, Wheel_Spin_Motor_I_Min, Wheel_Spin_Motor_I_Max, Wheel_Spin_Motor_KD, Wheel_Spin_Motor_KF, Wheel_Spin_Motor_ERROR_TOLERANCE);
PIDF Angle_Wheel2_pidf(PWM_Min, PWM_Max, Wheel_Spin_Motor_KP, Wheel_Spin_Motor_KI, Wheel_Spin_Motor_I_Min, Wheel_Spin_Motor_I_Max, Wheel_Spin_Motor_KD, Wheel_Spin_Motor_KF, Wheel_Spin_Motor_ERROR_TOLERANCE);

// Differential swerve modules
DifferentialSwerveModule module_front(COUNTS_PER_REV1, GEAR_Ratio, 45.0, 10.0);
DifferentialSwerveModule module_rear_left(COUNTS_PER_REV1, GEAR_Ratio, 45.0, 10.0);

// Wheel positions
float L = 0.3;
std::vector<std::pair<float, float>> wheel_positions = {
    {L, 0},                                     // Wheel 1: front
    {-L / 2, std::sqrt(3) * L / 2}             // Wheel 2: rear-left
};

void setup() {
    Serial.begin(115200);
    pinMode(Hall_Sensor1, INPUT_PULLUP);
    pinMode(Hall_Sensor2, INPUT_PULLUP);
    
    #ifdef MICROROS_WIFI
        IPAddress agent_ip(AGENT_IP);
        uint16_t agent_port = AGENT_PORT;
        set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip, agent_port);
    #else
        set_microros_serial_transports(Serial);
    #endif
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(300));
            }
            break;
        case AGENT_DISCONNECTED:
            MovePower(0, 0, 0, 0, 0, 0);
            destroyEntities();
            disconnect_count = 0;
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}

// Kinematics calculation for front and rear-left wheels
std::vector<std::pair<float, float>> kinematics(float Vx, float Vy, float omega) {
    std::vector<std::pair<float, float>> motor_speeds;

    for (size_t idx = 0; idx < wheel_positions.size(); ++idx) {
        float x = wheel_positions[idx].first;
        float y = wheel_positions[idx].second;

        float vix = Vx - omega * y;
        float viy = Vy + omega * x;

        float speed = hypot(vix, viy);
        float angle = viy == 0 && vix == 0 ? 0: atan2(viy, vix);

        motor_speeds.push_back({speed, angle});
    }

    return motor_speeds;
}

void calculate_Stering() {
    ticks_L_front = Encoder1.read();
    ticks_R_front = Encoder2.read();
    ticks_L_rear_left = Encoder3.read();
    ticks_R_rear_left = Encoder4.read();

    if(abs(V_x) <= 0.05 && abs(V_y) <= 0.05 && abs(Omega_z) <= 0.05) {
        setzero();
        return;
    }

    // Update angles
    module_front.update_angle(ticks_R_front, ticks_L_front);
    module_rear_left.update_angle(ticks_L_rear_left, ticks_R_rear_left);

    // Get current angles
    angle_front = NormalizeDegs(module_front.get_current_angle() + module_front.front_wheel_angle);
    angle_rear_left = NormalizeDegs(module_rear_left.get_current_angle() + module_rear_left.rear_left_wheel_angle);

    auto motor = kinematics(V_x, V_y, Omega_z);

    float speed_front_L_rpm = MPSToRPM(motor[0].first, WHEEL_DIAMETER);
    float speed_front_R_rpm = MPSToRPM(motor[0].first, WHEEL_DIAMETER);
    float speed_rearLeft_L_rpm = MPSToRPM(motor[1].first, WHEEL_DIAMETER);
    float speed_rearLeft_R_rpm = MPSToRPM(motor[1].first, WHEEL_DIAMETER);
    
    float angle1_correction = Angle_Wheel1_pidf.compute(motor[0].second * (180.0 / M_PI), angle_front);
    float angle2_correction = Angle_Wheel2_pidf.compute(motor[1].second * (180.0 / M_PI), angle_rear_left);

    float speed_front_L_pwm = front_L_pidf.compute(speed_front_L_rpm, rpm_front_L);
    float speed_front_R_pwm = front_R_pidf.compute(speed_front_R_rpm, rpm_front_R);
    float speed_rearLeft_L_pwm = rear_left_L_pidf.compute(speed_rearLeft_L_rpm, Encoder3.getRPM());
    float speed_rearLeft_R_pwm = rear_left_R_pidf.compute(speed_rearLeft_R_rpm, Encoder4.getRPM());

    float front_L_d = max(abs(speed_front_L_pwm) + abs(angle1_correction), (float) PWM_Max);
    float front_R_d = max(abs(speed_front_R_pwm) + abs(angle1_correction), (float) PWM_Max);
    float rearLeft_L_d = max(abs(speed_rearLeft_L_pwm) + abs(angle2_correction), (float) PWM_Max);
    float rearLeft_R_d = max(abs(speed_rearLeft_R_pwm) + abs(angle2_correction), (float) PWM_Max);

    float front_L_speed = ((speed_front_L_pwm - angle1_correction)/ front_L_d) * PWM_Max;
    float front_R_speed = ((speed_front_R_pwm + angle1_correction)/ front_R_d) * PWM_Max;
    float rearLeft_L_speed = ((speed_rearLeft_L_pwm - angle2_correction)/ rearLeft_L_d) * PWM_Max;
    float rearLeft_R_speed = ((speed_rearLeft_R_pwm + angle2_correction)/ rearLeft_R_d) * PWM_Max;

    MovePower(front_L_speed, front_R_speed,
              rearLeft_L_speed, rearLeft_R_speed,
              0, 0);

    // Debug messages
    debug_wheel_encoder_msg.linear.x = speed_front_L_pwm;
    debug_wheel_encoder_msg.linear.z = speed_front_R_pwm;
    debug_wheel_encoder_msg.linear.y = angle1_correction;
    debug_wheel_encoder_msg.angular.x = angle_front;
    debug_wheel_encoder_msg.angular.y = motor[0].second * (180.0 / M_PI);
    debug_wheel_encoder_msg.angular.z = V_x;
}

void read_hall_sensor() {
    hall_sensor1 = (digitalRead(Hall_Sensor1) == LOW);
    hall_sensor2 = (digitalRead(Hall_Sensor2) == LOW);

    hall_sensor1_msg.data = hall_sensor1;
    hall_sensor2_msg.data = hall_sensor2;
}

void setzero() {
    float search_speed_L = 750.0;
    float search_speed_R = -750.0;

    motor1.spin(search_speed_L);
    motor2.spin(search_speed_R);

    if (hall_sensor1) {
        motor1.spin(0);
        motor2.spin(0);
        Encoder1.reset();
        Encoder2.reset();
        ticks_L_front = 0;
        ticks_R_front = 0;
    }
}

void MovePower(float Motor1Speed, float Motor2Speed, float Motor3Speed, float Motor4Speed, float Motor5Speed, float Motor6Speed) {
    motor1.spin(Motor1Speed);
    motor2.spin(Motor2Speed);
    motor3.spin(Motor3Speed);
    motor4.spin(Motor4Speed);
}

void cmd_vel_callback(const void * msgin) 
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    V_x = msg->linear.x;
    V_y = msg->linear.y;
    Omega_z = msg->angular.z;
}
void wheelMoveCallback(const void *msgin)
{
    prev_cmd_time = millis();
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    motor1_target = moveMotor_msg.linear.x;
    motor2_target = moveMotor_msg.linear.y;
    motor3_target = moveMotor_msg.linear.z;
}

void movementModeCallback(const void *msgin)
{
    prev_cmd_time = millis();
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    movement_mode = movement_mode_msg.data.data;  // Copy it to global for later use
}


bool createEntities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // create node
    RCCHECK(rclc_node_init_default(&node, "differential_swerve_basemove_hardware", "", &support));

    // Pub
    RCCHECK(rclc_publisher_init_best_effort(
        &debug_move_wheel_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/wheel/motor_speed"));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_move_wheel_encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/wheel/encoder_rpm"));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_hall_sensor1_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "debug/hall_sensor1"));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_hall_sensor2_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "debug/hall_sensor2"));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_hall_sensor3_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "debug/hall_sensor3"));




    // RCCHECK(rclc_publisher_init_best_effort(
    //     &debug_move_wheel_angle_publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    //     "debug/wheel/angle"));

    // Sub

    RCCHECK(rclc_subscription_init_default(
        &arm_position_servo_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/servo_position"));

    RCCHECK(rclc_subscription_init_default(
        &move_wheel_motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/wheel/motor_speed"));

    RCCHECK(rclc_subscription_init_default(
        &movement_mode_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/movement_mode"));

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
        
    // RCCHECK(rclc_subscription_init_default(
    //     &wheel_angle_subscriber,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    //     "/wheel/angle"));

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 90;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        &controlCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_subscriber,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &move_wheel_motor_subscriber,
        &moveMotor_msg,
        &wheelMoveCallback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &arm_position_servo_subscriber,
        &arm_pos_msg,
        &Arm_position,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &movement_mode_subscriber,
        &movement_mode_msg,
        &movementModeCallback,
        ON_NEW_DATA));

    // synchronize time with the agent
    syncTime();

    return true;
}

void syncTime()
{
    // get the current time from the agent
        unsigned long now = millis();
        RCCHECK(rmw_uros_sync_session(10));
        unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}


bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_move_wheel_motor_publisher, &node);
    rcl_publisher_fini(&debug_move_wheel_encoder_publisher, &node);
    rcl_publisher_fini(&debug_hall_sensor1_publisher, &node);
    rcl_publisher_fini(&debug_hall_sensor2_publisher, &node);
    rcl_subscription_fini(&move_wheel_motor_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

// ROS2 callbacks and other functions remain the same as in the original code
// (cmd_vel_callback, wheelMoveCallback, movementModeCallback, controlCallback, 
// getEncoderData, publishData, createEntities, destroyEntities, etc.)