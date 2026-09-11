//#include "stm32f10x.h"                  // Device header
//#include "Motor.h"
//#include "PID.h"
//#include "Delay.h"
//#include "Car.h"
//#include "Grayscale_Sensor.h"


//PID_Controller tracking_pid;    // 寻迹专用的PID控制器


//#define DEFAULT_BASE_SPEED 40       // 默认基础前进速度（0-100）
//#define DEFAULT_MAX_CORRECTION 35   // 默认最大修正量限制

//static uint8_t tracking_enabled = 0;        // 寻迹模式使能标志
//static uint8_t base_speed = DEFAULT_BASE_SPEED;         // 当前基础速度
//static uint8_t max_correction = DEFAULT_MAX_CORRECTION; // 当前最大修正量

//static uint8_t gray_values[8] = {0};


//static float GetSensorPosition(void)
//{
//    uint8_t sensor_values[8];
//    float weighted_sum = 0.0f;
//    float sum = 0.0f;
//    static uint8_t detected_count = 0;
//    
//    // 直接调用您的传感器读取函数
//    GRAY_Analog_ReadReg(sensor_values);
//    
//    // 保存到全局变量
//    for(int i = 0; i < 8; i++) {
//        gray_values[i] = sensor_values[i];
//    }
//    // 计算加权位置（需要根据实际传感器特性调整阈值）
//	
//	
//    for(int i = 0; i < 8; i++) {
//        // 假设：数值小表示黑线，数值大表示白地
// float line_detected;
//        
//        if(sensor_values[i] < 60) {
//            line_detected = 1.0f;           // 确定黑线
//        } else if(sensor_values[i] > 180) {
//            line_detected = 0.0f;           // 确定白地
//    } else {
//            // 过渡区域：60→1.0, 180→0.0
//            line_detected = (180 - sensor_values[i]) / 120.0f;
//            line_detected = (line_detected > 1.0f) ? 1.0f : (line_detected < 0.0f) ? 0.0f : line_detected;
//        }
//    
//	if(line_detected > 0.1f) detected_count++;
//        
//        weighted_sum += line_detected * (i - 3.5f);
//        sum += line_detected;
//    }
//    
//    // 关键改进：根据检测到的传感器数量调整灵敏度
//    if(detected_count >= 1) {
//        float position = weighted_sum / sum;
//        
//        // 如果只有2个传感器检测到，增强灵敏度
//        if(detected_count == 2) {
//            position *= 1.5f;  // 增加50%的灵敏度
//        }
//        
//        // 限制最大位置值
//        if(position > 3.0f) position = 3.0f;
//        if(position < -3.0f) position = -3.0f;
//        
//        return position;
//    } else {
//        // 脱线处理
//        static float last_position = 0.0f;
//        static uint8_t lost_count = 0;
//        
//        lost_count++;
//        if(lost_count < 6) {
//            return last_position;
//        } else {
//            return (last_position > 0) ? 1.2f : -1.2f;
//			}
//		}
//	}
//void TrackingCar_Init(void)
//{
//	Car_Init();
//	GRAY_Init();
//	 
//	

//	tracking_enabled = 0;
//	base_speed = DEFAULT_BASE_SPEED;
//	max_correction = DEFAULT_MAX_CORRECTION;
//}

//void TrackingCar_Update(void)
//{
//	 if(!tracking_enabled) return;
//	
//	float position = GetSensorPosition();     //获取传感器数据
//	
//	float correction = PID_Calculate(&tracking_pid, 0.0f, position);  //PID控制计算

//	//int8_t left_speed = base_speed -correction;     
//	//int8_t right_speed = base_speed + correction;   
//	
//	int8_t left_speed = 30;
//    int8_t right_speed = 30;
////	if(position < -0.5) {
////        // 向右转 + PID微调
////        float correction = PID_Calculate(&tracking_pid, 0.0f, position);    
////        left_speed = 40 -correction * 0.5f;   // 减小PID影响             //
////        right_speed = 10 + correction * 0.5f;                             //
////    } 
////    else if(position > 0.5) {
////        // 向左转 + PID微调
////        float correction = PID_Calculate(&tracking_pid, 0.0f, position);
////        left_speed = 10 - correction * 0.5f;                        //
////        right_speed = 40 + correction * 0.5f;                       //
////    }
////    else {
////        // 直行 + PID微调
////        float correction = PID_Calculate(&tracking_pid, 0.0f, position);
////        left_speed = 30 - correction;                              //
////        right_speed = 30 + correction;                             //
////    }
//		if(position < -0.5) {
//        // 向右转 + PID微调
//        left_speed = 10 -correction * 0.5f;   // 减小PID影响             //
//        right_speed = 60 + correction * 0.5f;                             //
//    } 
//    else if(position > 0.5) {
//        // 向左转 + PID微调
//        left_speed = 60 - correction * 0.5f;                        //
//        right_speed = 10 + correction * 0.5f;                       //
//    }
//    else {
//        // 直行 + PID微调
//        left_speed = 20 - correction;                              //
//        right_speed = 20 + correction;                             //
//    }
//	
//	if(left_speed > 100) left_speed = 100;
//    if(left_speed < -100) left_speed = -100;
//    if(right_speed > 100) right_speed = 100;
//    if(right_speed < -100) right_speed = -100;
//	
//	
//	leftSpeed(left_speed);   // 设置左轮速度
//    rightSpeed(right_speed); // 设置右轮速度


//    

////    float position = GetSensorPosition();
////    
////    // 使用简化的条件控制
////    int8_t left_speed = 30;  // 基础速度
////    int8_t right_speed = 30;
////    
////    if(position < -0.5) {
////        // 向右转
////        left_speed = 40;
////        right_speed = 10;
////    } 
////    else if(position > 0.5) {
////        // 向左转
////        left_speed = 10;
////        right_speed = 40;
////	}
////    leftSpeed(left_speed);
////    rightSpeed(right_speed);
//}
//	


//void TrackingCar_Start(void)
//{
//    tracking_enabled = 1;        // 使能寻迹模式
//    PID_Reset(&tracking_pid);    // 重置PID控制器，清除历史数据
//}


//void TrackingCar_Stop(void)
//{
//    tracking_enabled = 0;    // 禁用寻迹模式
//    Car_Stop();             // 停止所有电机
//}

//uint8_t TrackingCar_IsRunning(void)
//{
//    return tracking_enabled;
//}


//void TrackingCar_SetPIDParams(float kp, float ki, float kd)
//{
//    PID_SetParameters(&tracking_pid, kp, ki, kd);
//}



//void TrackingCar_SetBaseSpeed(uint8_t speed)
//{
//    // 限制速度在合理范围内
//    if(speed > 100) speed = 100;
//    base_speed = speed;  // 更新基础速度
//}





