#include "Gyro.h"
#define Gyro_scale_factor    4.702317
#define MAX_Angular_velocity 2147483647
#define Temperature_Coefficient 16
#define wait_time 15

uint8_t Gyro_Msg[9];
Gyro_Info_t Gyro_Infomation;
double Angle_num;
bool time_over;

static double Static_deviation;
static uint8_t Validation;
static uint8_t Gyro_validation[7];

void Gyro_init(void)
{
    Static_deviation = 0.0;
    Angle_num = 0.0;
    time_over = 0;
}

int Gyro_Data_validation(void)
{
    Gyro_validation[0] = ((Gyro_Msg[0] & 0x40) ^ (Gyro_Msg[1] & 0x40) ^ (Gyro_Msg[2] & 0x40) ^ (Gyro_Msg[3] & 0x40) ^ (Gyro_Msg[4] & 0x40));
    Gyro_validation[1] = ((Gyro_Msg[0] & 0x20) ^ (Gyro_Msg[1] & 0x20) ^ (Gyro_Msg[2] & 0x20) ^ (Gyro_Msg[3] & 0x20) ^ (Gyro_Msg[4] & 0x20));
    Gyro_validation[2] = ((Gyro_Msg[0] & 0x10) ^ (Gyro_Msg[1] & 0x10) ^ (Gyro_Msg[2] & 0x10) ^ (Gyro_Msg[3] & 0x10) ^ (Gyro_Msg[4] & 0x10));
    Gyro_validation[3] = ((Gyro_Msg[0] & 0x08) ^ (Gyro_Msg[1] & 0x08) ^ (Gyro_Msg[2] & 0x08) ^ (Gyro_Msg[3] & 0x08) ^ (Gyro_Msg[4] & 0x08));
    Gyro_validation[4] = ((Gyro_Msg[0] & 0x04) ^ (Gyro_Msg[1] & 0x04) ^ (Gyro_Msg[2] & 0x04) ^ (Gyro_Msg[3] & 0x04) ^ (Gyro_Msg[4] & 0x04));
    Gyro_validation[5] = ((Gyro_Msg[0] & 0x02) ^ (Gyro_Msg[1] & 0x02) ^ (Gyro_Msg[2] & 0x02) ^ (Gyro_Msg[3] & 0x02) ^ (Gyro_Msg[4] & 0x02));
    Gyro_validation[6] = ((Gyro_Msg[0] & 0x01) ^ (Gyro_Msg[1] & 0x01) ^ (Gyro_Msg[2] & 0x01) ^ (Gyro_Msg[3] & 0x01) ^ (Gyro_Msg[4] & 0x01));
    Validation = ((Gyro_validation[0] | Gyro_validation[1] | Gyro_validation[2] | Gyro_validation[3] | Gyro_validation[4] | Gyro_validation[5] | Gyro_validation[6]) & 0x7F);
    if(Validation == Gyro_Msg[5])
        return 1;
    else
        return 0;
}

int Gyro_Temperature_validation()
{
    Gyro_validation[0] = ((Gyro_Msg[6] & 0x40) ^ (Gyro_Msg[7] & 0x40));
    Gyro_validation[1] = ((Gyro_Msg[6] & 0x20) ^ (Gyro_Msg[7] & 0x20));
    Gyro_validation[2] = ((Gyro_Msg[6] & 0x10) ^ (Gyro_Msg[7] & 0x10));
    Gyro_validation[3] = ((Gyro_Msg[6] & 0x08) ^ (Gyro_Msg[7] & 0x08));
    Gyro_validation[4] = ((Gyro_Msg[6] & 0x04) ^ (Gyro_Msg[7] & 0x04));
    Gyro_validation[5] = ((Gyro_Msg[6] & 0x02) ^ (Gyro_Msg[7] & 0x02));
    Gyro_validation[6] = ((Gyro_Msg[6] & 0x01) ^ (Gyro_Msg[7] & 0x01));
    Validation = ((Gyro_validation[0] | Gyro_validation[1] | Gyro_validation[2] | Gyro_validation[3] | Gyro_validation[4] | Gyro_validation[5] | Gyro_validation[6]) & 0x7F);
    if(Validation == Gyro_Msg[8])
        return 1;
    else
        return 0;
}

void Gyro_Info(void)
{
    if((Gyro_Msg[4] & 0x08) == 0x08) {
        //negative
        Gyro_Infomation.Gyro_Message = -((int32_t)(~((Gyro_Msg[0]  |  Gyro_Msg[1] << 7  |  Gyro_Msg[2] << 14  |  Gyro_Msg[3] << 21  |  Gyro_Msg[4] << 28) & 0x7FFFFFFF)) + 1 + MAX_Angular_velocity);
    } else {
        //positive
        Gyro_Infomation.Gyro_Message = (int32_t)((Gyro_Msg[0]  |  Gyro_Msg[1] << 7  |  Gyro_Msg[2] << 14  |  Gyro_Msg[3] << 21  |  Gyro_Msg[4] << 28) & 0x7FFFFFFF);
    }
    Gyro_Infomation.Angular_velocity_z = ((double)(Gyro_Infomation.Gyro_Message) / Gyro_scale_factor / 10000.0) - Static_deviation;
    Gyro_Infomation.Gyro_Temperature = (double)((Gyro_Msg[6]  |  Gyro_Msg[7] << 7) & 0x3FFF) / Temperature_Coefficient;
    printf("the Gyrp message is %d\nthe angular velocity of 'z' is %lf\nthe Gyro Temperature is %f\n\n", Gyro_Infomation.Gyro_Message, Gyro_Infomation.Angular_velocity_z, Gyro_Infomation.Gyro_Temperature);
}

double GyroGetAngularVelocityZ(void)
{
    return Gyro_Infomation.Angular_velocity_z;
}

float GyroGetTemperature(void)
{
    return Gyro_Infomation.Gyro_Temperature;
}

static void Static_deviation_Cal(void * arg)
{
    double Static_deviation_accumulation = 0;
    for(int i = 0; i < wait_time * 100; i++) {
        Static_deviation_accumulation += GyroGetAngularVelocityZ();
        vTaskDelay(10 / portMAX_DELAY);
    }
    Static_deviation = Static_deviation_accumulation / (wait_time * 100);
    time_over = 1;
    vTaskDelete(NULL);
}

void Deviation_Cal(void)
{
    xTaskCreate(Static_deviation_Cal, "Static_deviation_Cal", 1024 * 2, NULL, 7, NULL);
}