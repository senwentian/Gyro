#ifndef _GYRO_H
#define _GYRO_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "stdio.h"

typedef struct {
    double Angular_velocity_z;
    int32_t Gyro_Message;
    float Gyro_Temperature;
} Gyro_Info_t;

extern Gyro_Info_t Gyro_Infomation;
extern uint8_t Gyro_Msg[9];
extern double Angle_num;
extern bool time_over;

void Gyro_init(void);

int Gyro_Data_validation(void);

int Gyro_Temperature_validation();

void Gyro_Info(void);

double GyroGetAngularVelocityZ(void);

float GyroGetTemperature(void);

void Deviation_Cal(void);

#endif
