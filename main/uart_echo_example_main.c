#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "led_strip.h"
static const int RX_BUF_SIZE = 1024;

//uart tx和rx, 可以修改
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

#define RMT_TX_CHANNEL RMT_CHANNEL_0
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}
//串口初始化
void uart_init(void) {
    //uart配置参数
    const uart_config_t uart_config = {
        //波特率
        .baud_rate = 115200,
        //数据位
        .data_bits = UART_DATA_8_BITS,
        //校验位
        .parity = UART_PARITY_DISABLE,
        //停止位
        .stop_bits = UART_STOP_BITS_1,
        //硬件流控模式
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //时钟源
        .source_clk = UART_SCLK_APB,
    };
    //安装uart驱动, 配置UART RX 环形缓冲区大小, 不设置TX 环形缓冲区, 不设置UART 事件队列, 不设置中断
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    //配置uart参数
    uart_param_config(UART_NUM_1, &uart_config);
    //配置uart引脚
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

//uart 发送线程
static void tx_task(void *arg)
{
    static uint8_t data[]={0x01,0x02,0x03,0x04,0x05};
    while (1) {
        //往UART_NUM_1发送"Hello world"
        //uart_write_bytes(UART_NUM_1, "Hello world", strlen("Hello world"));
        uart_write_bytes(UART_NUM_1,  data, sizeof(data));
        //延时2000ms
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

//uart 接收线程
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    //动态分配内存data
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        //从UART_NUM_1接收数据到data, 最大接收RX_BUF_SIZE个数据, 超时时长1000ms
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            //字符串结束符0
            data[rxBytes] = 0;
            //字符串打印接收数据
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            //16进制打印接收数据
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    //动态释放内存data
    free(data);
}

//main启动
void app_main(void)
{


    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(8, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "LED Rainbow Chase Start");
    //串口初始化
    uart_init();
   
    //创建并启动串口接收线程rx_task
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //创建并启动串口发送线程tx_task
   xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
