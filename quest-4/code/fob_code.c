/*
Contributors: Raghurama Bukkarayasamudram, Ritam Das, Brian Macomber
Date: 11/13/2020
Quest 4 - E-Voting v2
*/
/*

   RMT Pulse          -- pin 26 -- A0
   UART Transmitter   -- pin 25 -- A1
   UART Receiver      -- pin 34 -- A2

   Button 1 (Hardware interrupt) -- pin 4 -- A5
   Button 2 (Send IR) -- pin 27
   Leader Indicator       -- pin 13 -- Onboard LED

   Red LED            -- pin 33
   Green LED          -- pin 32
   Blue LED           -- pin 14

   Features:
   - Sends UART payload -- | START | myColor | myID | Checksum? |
   - Outputs 38kHz using RMT for IR transmission
   - Onboard LED blinks device ID (myID)
   - Button press to change device ID
   - RGB LED shows traffic light state (red, green, yellow)
   - Timer controls leader election algorithm
*/
////////////////////////////////////////////////////////////////////////////////
//                              LIBRARIES                                     //
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include <sys/param.h>
#include <lwip/netdb.h>
#include "esp_types.h"
#include "freertos/queue.h"
#include "esp_vfs_dev.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/event_groups.h"

////////////////////////////////////////////////////////////////////////////////
//                              DEFINITIONS                                   //
////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////
// RMT definitions
#define RMT_TX_CHANNEL 1                                 // RMT channel for transmitter
#define RMT_TX_GPIO_NUM 25                               // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV 100                                  // RMT counter clock divider
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US 9500                       // RMT receiver timeout value(us)

///////////////////////////////////////////
// Button / UART / LED definitions
// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1 4
#define BUTTON2 27
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_INPUT_IO_1
int sendFlag = 0;

// LED Output pins definitions
#define BLUEPIN 14
#define GREENPIN 32
#define REDPIN 15
#define ONBOARD 13

///////////////////////////////////////////
// UDP communication Definitions
#define PORT 1131
#define NODE_IP_ADDR "192.168.7.196"

//used for array definitions
#define MAX 50
// this is a placeholder, itll be dynamically set inside of the client task
char HOST_IP_ADDR[MAX] = "192.168.7.245";

//Used in UDP comms
static const char *TAG = "example";
static char payload[30];

//udp server vars
char recv_status[MAX];
char recv_ID = '9'; //start at a high number so the first ESP will set itself to leader upon not recieving anything
char recv_deviceAge[MAX];
char recv_leaderHeartbeat[MAX];
char tempBuffer[MAX];
int irMsgFlag = 0;

//Sending and recieveing flags for UDP and IR
int IRSentFlag = 0;
int RecvFlag = 0;
int UDPFlag = 0;

///////////////////////////////////////////
//Timer Definitions
#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds
#define TIMER_INTERVAL_1_SEC (1)
#define TEST_WITH_RELOAD 1 // Testing will be done with auto reload
#define ELECTION_TIMEOUT 6
#define LEADER_TIMEOUT 30
#define HEARTBEAT 1
#define UDP_TIMER 3
int timeout = LEADER_TIMEOUT;
int udpTimer = UDP_TIMER;

// A simple structure to pass "events" to main task
typedef struct
{
    int flag; // flag for enabling stuff in timer task
} timer_event_t;

/////////////////////////////////////////////
// Parameters sent from ESP to ESP (UDP)
// for keeping a leader elected
// (this also include myID (declared below)
char status[10] = "No_Leader";
char deviceAge[4] = "New";        // either New or Old
char leaderHeartbeat[6] = "Dead"; //either Dead or Alive
typedef enum
{
    ELECTION_STATE,
    LEADER_STATE,
    FOLLOWER_STATE
} state_e;

state_e deviceState = FOLLOWER_STATE;

// To keep track of current leader at all times
char leaderIP[MAX] = "";

#define NUM_FOBS 2
char IPtable[NUM_FOBS][20] = {
    "192.168.7.245",  // FOB 1
    "192.168.7.176"}; // FOB 2

//Voter Variables
char voterVote = 'G';
char voterID = '1';

///////////////////////////////////////////
// IR Transmission Variables            //
// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
char myColor = 'R';
char myID = '0'; // current ESP
int len_out = 4;

// Used in LED task and button1 task - increments to represent color
int colorID = 0;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle timer_queue;

// System tags
static const char *TAG_SYSTEM = "system";

///////////////////////////////////////////
/// WiFi Definitions                     //
#define EXAMPLE_ESP_WIFI_SSID "SUPREME-WiFi"
#define EXAMPLE_ESP_WIFI_PASS "gh7a84VSeW7TgYC"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
static EventGroupHandle_t s_wifi_event_group;
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *WIFI_TAG = "wifi station";
static int s_retry_num = 0;

////////////////////////////////////////////////////////////////////////////////
//                        INIT / HELPER FUNCTIONS                             //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Utilities ///////////////////////////////////////////////////////////////////

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para)
{

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Checksum
char genCheckSum(char *p, int len)
{
    char temp = 0;
    for (int i = 0; i < len; i++)
    {
        temp = temp ^ p[i];
    }
    // printf("%X\n",temp);

    return temp;
}
bool checkCheckSum(char *p, int len)
{
    char temp = (char)0;
    bool isValid;
    for (int i = 0; i < len - 1; i++)
    {
        temp = temp ^ p[i];
    }
    // printf("Check: %02X ", temp);
    if (temp == p[len - 1])
    {
        isValid = true;
    }
    else
    {
        isValid = false;
    }
    return isValid;
}
////////////////////////////////////////////////////////////////////////////////
// Init Functions //////////////////////////////////////////////////////////////
// Wifi Init functions
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(WIFI_TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(WIFI_TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void initalizeWiFi()
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
}

// RMT tx init
static void rmt_tx_init()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init()
{
    // Basic configs
    uart_config_t uart_config = {
        .baud_rate = 1200, // Slow BAUD rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart_config);

    // Set UART pins using UART0 default pins
    uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Reverse receive logic line
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

    // Install UART driver
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs
static void led_init()
{
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_pad_select_gpio(ONBOARD);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ONBOARD, GPIO_MODE_OUTPUT);
}

// Configure timer - sets 1 second timer
static void alarm_init()
{
    // Select and initialize basic parameters of the timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_1_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
                       (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// Button interrupt init
static void button_init()
{
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *)GPIO_INPUT_IO_1);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task

    // other button init
    gpio_pad_select_gpio(BUTTON2);
    gpio_set_direction(BUTTON2, GPIO_MODE_INPUT);
}

////////////////////////////////////////////////////////////////////////////////
//                            Tasks                                           //
////////////////////////////////////////////////////////////////////////////////
// Button task -- rotate through colorIDs
void button_task()
{
    uint32_t io_num;
    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            xSemaphoreTake(mux, portMAX_DELAY);
            if (colorID == 2)
            {
                colorID = 0;
            }
            else
            {
                colorID++;
            }
            xSemaphoreGive(mux);
            printf("Button pressed.\tcolorID: %i\n", colorID);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

//Button 2 task - when it's pressed data is sent over IR
void button_2_task()
{
    int button_flag;
    while (1)
    {
        button_flag = gpio_get_level(BUTTON2);
        if (!button_flag)
        {
            printf("button 2 pressed\n");
            sendFlag = 1;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        sendFlag = 0;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// IR Send and Recieve Task
void IR_task()
{
    // Buffer for input data
    char *data_in = (char *)malloc(len_out);
    while (1)
    {
        if (sendFlag && deviceState != LEADER_STATE) // cannot send over IR as the leader, since it will just be sent directly to the node server
        {
            char *data_out = (char *)malloc(len_out);
            xSemaphoreTake(mux, portMAX_DELAY);
            // printf("myID is %c \n", myID[0]);
            data_out[0] = start;
            data_out[1] = myColor;
            data_out[2] = myID;
            data_out[3] = genCheckSum(data_out, len_out - 1);

            // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_out, len_out, ESP_LOG_INFO);

            for (int i = 0; i < 5; i++)
            {
                uart_write_bytes(UART_NUM_1, data_out, len_out);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            sendFlag = 0;
            printf("data sucessfully sent\n");
            xSemaphoreGive(mux);
        }
        else
        {
            int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
            if (len_in > 0)
            {
                if (data_in[0] == start)
                {
                    if (checkCheckSum(data_in, len_out))
                    {
                        //We recieved a vote through IR!
                        ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);

                        voterVote = data_in[1];
                        voterID = data_in[2];
                        RecvFlag = 1;
                        printf("ir data recieved: \t");
                        printf("voter: %c vote: %c \n", voterID, voterVote);
                    }
                }
            }
            else
            {
                // printf("Nothing received.\n");
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    free(data_in);
}

// LED task to light LED current Color STate
void led_task()
{
    while (1)
    {
        switch (colorID)
        {
        case 0: // Red
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 1);
            gpio_set_level(BLUEPIN, 0);
            myColor = 'R';
            // printf("Current state: %c\n",status);
            break;
        case 1: // Green
            gpio_set_level(GREENPIN, 1);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 0);
            myColor = 'G';
            // printf("Current state: %c\n",status);
            break;
        case 2: // Blue
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 1);
            myColor = 'B';
            // printf("Current state: %c\n",status);
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// LED task to flag onboard LED if the device is the Poll Leader
void id_task()
{
    while (1)
    {
        ///////////////////
        // STATE MACHINE //
        ///////////////////

        // Check device state and handle incoming data accordingly
        // int myID_num = atoi(myID);
        // int recv_ID_num = atoi(recv_ID);
        if (deviceState == ELECTION_STATE)
        {
            printf("ELECTION STATE\n");
            gpio_set_level(ONBOARD, 0);
            //once part of an election, you are no longer new
            strcpy(deviceAge, "OLD");
            strcpy(status, "No_Leader"); // Status is "No_Leader"

            if (myID < recv_ID)
            {
                printf("Switching to Leader state\n");
                strcpy(status, "Leader"); // Status is "No_Leader"
                strcpy(leaderHeartbeat, "Alive");
                deviceState = LEADER_STATE; // Stay in election state
                timeout = ELECTION_TIMEOUT; // Reset election timeout
            }
            else if (myID > recv_ID)
            {
                printf("Switching to follower state\n");
                timeout = LEADER_TIMEOUT; // Change timeout variable to leader timeout constant
                udpTimer = UDP_TIMER * 2;
                deviceState = FOLLOWER_STATE; // Change to follower state
            }

            if (timeout <= 0)
            {
                printf("Switching to Leader state\n");
                deviceState = LEADER_STATE;
            }
        }
        else if (deviceState == FOLLOWER_STATE)
        {
            // printf("FOLLOWER STATE\n");
            gpio_set_level(ONBOARD, 0);
            // udpTimer = UDP_TIMER * 2;
            // strcpy(transmitting, "No"); // Stop transmitting
            if (strcmp(recv_deviceAge, "New") == 0)
            {
                printf("Switching to election state\n");
                deviceState = ELECTION_STATE; // Change to election state
                timeout = ELECTION_TIMEOUT;   // Change timeout variable to election timeout constant
            }
            else if (strcmp(recv_leaderHeartbeat, "Alive") == 0) //leader is still alive
            {
                timeout = LEADER_TIMEOUT; // Reset leader timeout upon receiving leader heartbeat
                strcpy(status, "Leader"); // Update status to leader upon receiving leader heartbeat
                printf("leaderIP is originally %s \n", leaderIP);
                strcpy(leaderIP, IPtable[atoi(&recv_ID)]);
                printf("leaderIP is now changed to %s with the recv_ID of %c \n", leaderIP, recv_ID);
            }
            else if (timeout <= 0)
            {
                printf("Switching to election state\n");
                deviceState = ELECTION_STATE; // Change to election state
                timeout = ELECTION_TIMEOUT;   // Change timeout variable to election timeout constant
            }
        }
        else if (deviceState == LEADER_STATE)
        {
            gpio_set_level(ONBOARD, 1);
            // handles the leader heartbeat every 1 second
            if (udpTimer < 0)
            {
                udpTimer = HEARTBEAT;
            }
            if (strcmp(recv_deviceAge, "New") == 0)
            {
                printf("Switching to election state\n");
                deviceState = ELECTION_STATE; // Change state to election state
                timeout = ELECTION_TIMEOUT;   // Change timeout variable to election timeout constant
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Handles what happens when the 1 second timer goes off
static void timer_evt_task(void *arg)
{
    while (1)
    {
        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Do something if triggered!
        if (evt.flag == 1)
        {
            timeout--;
            udpTimer--;
            printf("timeout: %d\tudptimer: %d\n", timeout, udpTimer);
        }
    }
}

// UDP server
static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1)
    {
        if (addr_family == AF_INET)
        {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        }
        else if (addr_family == AF_INET6)
        {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6)
        {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1)
        {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0)
            {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else
            {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET)
                {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                }
                else if (source_addr.sin6_family == PF_INET6)
                {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...

                strcpy(tempBuffer, rx_buffer);
                char *token = strtok(rx_buffer, ",");

                for (int i = 0; i < 4; i++)
                {
                    if (i == 0)
                    {
                        strcpy(recv_status, token);
                    }
                    else if (i == 1)
                    {
                        if (strcmp(token, "R") == 0 || strcmp(token, "B") == 0)
                        {
                            irMsgFlag = 1;
                            break;
                        }
                        recv_ID = *token;
                    }
                    else if (i == 2)
                    {
                        strcpy(recv_deviceAge, token);
                    }
                    else if (i == 3)
                    {
                        strcpy(recv_leaderHeartbeat, token);
                    }

                    token = strtok(NULL, ",");
                }
                char *token2 = strtok(tempBuffer, ",");

                if (irMsgFlag == 1)
                {
                    while (token2 != NULL)
                    {
                        if (strcmp(token2, "B") == 0)
                        {
                            voterVote = 66;
                        }
                        else if (strcmp(token2, "R") == 0)
                        {
                            voterVote = 82;
                        }
                        else if (strcmp(token2, "0") == 0)
                        {
                            voterID = 48;
                        }
                        else if (strcmp(token2, "1") == 0)
                        {
                            voterID = 49;
                        }
                        // voterID = token2;
                        token2 = strtok(NULL, ",");
                    }
                    // Set flags back
                    irMsgFlag = 0;
                    RecvFlag = 1;
                }

                //ERROR CHECKING
                int err = sendto(sock, payload, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void
udp_client_task(void *pvParameters)
{
    int addr_family = 0;
    int ip_protocol = 0;

    while (1)
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1)
        {

            // Here is where all the UDP messaging is handled
            if (udpTimer <= 0) // Based on 3 second (leader) or 6 second (follower) itll ping all the other fobs
            {
                for (int i = 0; i < NUM_FOBS; i++)
                {
                    // don't send status pings to yourself
                    if (IPtable[i] != IPtable[atoi(&myID)])
                    {
                        strcpy(HOST_IP_ADDR, IPtable[i]);
                        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);

                        //data sent in form:    "status,myID,deviceAge,leaderHeartBeat"
                        //  where status is if there is a leader elected or not
                        //  where myID is the device that is sending it
                        //  where deviceAge is if the ESP is newly connected to the system or part of it

                        sprintf(payload, "%s,%c,%s,%s", status, myID, deviceAge, leaderHeartbeat);
                        printf("Sending a ping to a nearby ESP...\n");

                        //Send data to other ESP
                        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                        strcpy(deviceAge, "Old");

                        if (err < 0)
                        {
                            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                            break;
                        }
                        ESP_LOGI(TAG, "Message sent");

                        // reset UDP timer
                        udpTimer = UDP_TIMER;
                    }
                }
            }
            if (RecvFlag == 1 && deviceState == LEADER_STATE) //Leader state and recieved an IR vote from someone else
            {
                dest_addr.sin_addr.s_addr = inet_addr(NODE_IP_ADDR);
                printf("Sending to the node server.\n");

                //put the id and the vote in the payload to send in the form ID,VOTE
                sprintf(payload, "%c,%c", voterID, voterVote);
                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                strcpy(deviceAge, "Old");

                // reset IR flag
                RecvFlag = 0;
            }
            else if (RecvFlag == 1 && deviceState != LEADER_STATE) //Non leader state and recieved an IR vote
            {
                //get poll leader IP and send the vote to the poll leader
                dest_addr.sin_addr.s_addr = inet_addr(leaderIP);
                printf("Sending to poll leader.\n");

                //put the id and the vote in the payload to send in the form ID,VOTE
                sprintf(payload, "%c,%c", voterID, voterVote);
                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                strcpy(deviceAge, "Old");

                // reset IR flag
                RecvFlag = 0;
            }
            else if (sendFlag == 1 && deviceState == LEADER_STATE) //Sending Vote to node server as the leader
            {
                dest_addr.sin_addr.s_addr = inet_addr(NODE_IP_ADDR);
                printf("Sending to the node server.\n");

                //put the id and the vote in the payload to send in the form ID,VOTE
                sprintf(payload, "%c,%c", voterID, voterVote);
                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                strcpy(deviceAge, "Old");

                // Reset send button flag
                sendFlag = 0;
            }
            // error handling for a bad send
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            printf("sending to ip addess %s \n", HOST_IP_ADDR);
            ESP_LOGI(TAG, "Message sent");

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        vTaskDelete(NULL);
    }
}

////////////////////////////////////////////////////////////////////////////////
//                              Main                                          //
////////////////////////////////////////////////////////////////////////////////
void app_main()
{
    // initalize the wifi
    initalizeWiFi();
    //wait a little for wifi to finish
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // Mutex for current values when sending
    mux = xSemaphoreCreateMutex();

    // Create a FIFO queue for timer-based events
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    // Create task to handle timer-based events
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, configMAX_PRIORITIES, NULL);

    // Initialize all the things
    alarm_init();
    uart_init();
    button_init();
    rmt_tx_init();
    led_init();

    // Create tasks for:
    //    IR recieve and send,
    //    detect button 1,
    //    detect button 2,
    //    UDP server,
    //    UDP Client,
    //    Onboard Leader indicator
    xTaskCreate(id_task, "set_id_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button_task, "button_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button_2_task, "button_2_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(IR_task, "IR_task", 1024 * 4, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(led_task, "set_traffic_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(udp_server_task, "udp_server", 4096, (void *)AF_INET, configMAX_PRIORITIES, NULL);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
}