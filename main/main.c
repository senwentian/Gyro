#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "Gyro.h"

QueueHandle_t uart1_queue;
static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 10, 0, 40, &uart1_queue, 0);
    ESP_LOGI("Project", "UART Init is success");
}

static void Test_Task(void *arg)
{
    int flag = 0;
    while (1) {
        char *pbuffer = (char *)calloc(1, 700);
        if(flag == 0){
            vTaskGetRunTimeStats(pbuffer);
            ESP_LOGI("***************Task_Run_Time_List***************", "\n\nName            Total Time      Percentage \n%s", pbuffer);
        }else{
           vTaskList(pbuffer);
           ESP_LOGI("***************Task_State_List***************", "\n\nName          State  Priority Stackleft Number CoreID\n%s", pbuffer);
        }
        flag = !flag;
        free(pbuffer);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

static void rx_task(void *arg)
{
    int read_len;
    uart_event_t event;
    ESP_LOGI("UART", "Hello_world");
    uint8_t* dtmp = (uint8_t*) malloc(1024);
    for(;;) {
        uint8_t flag = 1;
        //Waiting for UART event.
        xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY);
        bzero(dtmp, 1024);
        //ESP_LOGI("UART", "uart[%d] event:", UART_NUM_1);
        switch(event.type) {
            //Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                uart_get_buffered_data_len(UART_NUM_1, (size_t*)&read_len);
                read_len = uart_read_bytes(UART_NUM_1, dtmp, read_len, portMAX_DELAY);
                //ESP_LOGI("UART", "Read %d bytes: %d, %d, %d, %d, %d", read_len, *dtmp, *(dtmp+1), *(dtmp+2), *(dtmp+3), *(dtmp+4));
                dtmp[read_len] = 0;
                const uint8_t *d = dtmp;
                static uint8_t RxState = 0, RxDataIndex = 0;
                while(flag){
                    switch(RxState){
                        case 0:
                            if(*d == 0x80) {
                                RxState++;
                            } else
                                RxState = 0;
                            break;

                        case 1:
                            Gyro_Msg[RxDataIndex] = *d;
                            RxDataIndex++;
                            if(RxDataIndex >= 6) {
                                int Check_t = Gyro_Data_validation();
                                if(Check_t == 1) {
                                    RxState++;
                                } else {
                                    RxState = 0;
                                    RxDataIndex = 0;
                                }
                            }
                            break;

                            case 2:
                                Gyro_Msg[RxDataIndex] = *d;
                                RxDataIndex++;
                                if(RxDataIndex >= 9) {
                                    int Check_t = Gyro_Temperature_validation();
                                    if(Check_t == 1) {
                                        Gyro_Info();
                                        RxState = 0;
                                        flag = 0;
                                    } else {
                                        RxState = 0;
                                        RxDataIndex = 0;
                                    }
                                }
                                break;
                            
                            default:
                                RxState = 0;
                    }
                    d++;
                }
                break;

            default:
                ESP_LOGI("UART", "uart event type: %d", event.type);
                break;
        }
    }
    free(dtmp);
    dtmp = NULL;
}

void Angle_Test(void * arg)
{
    while (1)
    {
        if(time_over == 1) {
            Angle_num += GyroGetAngularVelocityZ();
            ESP_LOGI("Angle", "the Angle is %lf", Angle_num);
        }
        vTaskDelay(15 / portMAX_DELAY);
    }
}

void app_main(void)
{
    init();
    Gyro_init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 4, NULL, 6, NULL);
    Deviation_Cal();
    // xTaskCreate(Angle_Test, "Angle_Test", 1024 *2, NULL, 5, NULL);
    // xTaskCreate((TaskFunction_t)Test_Task, "Test_Task", 1024*2, NULL, 5, NULL);
}
