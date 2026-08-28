/*
    Codigo Encoder Rotativo ky-040
    *Implementando Micro Ros- FULGOR

 */

#include <unistd.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "driver/pulse_cnt.h" //driver del periférico PCNT
#include "driver/gpio.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "ENCODER";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define MICRO_ROS_APP_STACK      16000
#define MICRO_ROS_APP_TASK_PRIO  5

// Define los límites superior e inferior del contador antes de reiniciar o desbordar
#define PCNT_HIGH_LIMIT 100
#define PCNT_LOW_LIMIT  -100

// Pines del encoder
#define GPIO_A 32 // CLK
#define GPIO_B 33 // DT

// TODO: calibrar girando el eje exactamente una vuelta y viendo cuánto acumuló total_ticks
#define TICKS_PER_REV     80.0f
#define PUBLISH_PERIOD_MS 20

static pcnt_unit_handle_t pcnt_unit = NULL;
static volatile int64_t total_ticks = 0;  // ticks acumulados por wrap-around de hardware (±100)

static rcl_publisher_t ticks_publisher;
static rcl_publisher_t rpm_publisher;
static std_msgs__msg__Int32 ticks_msg;
static std_msgs__msg__Float32 rpm_msg;

static int32_t last_ticks = 0;
static int64_t last_time_us = 0;

// Callback de hardware: se dispara cuando el contador toca el límite alto o bajo,
// tener una posición absoluta continua en vez de un valor que se resetea cada 100 ticks.
static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    if (edata->watch_point_value == PCNT_HIGH_LIMIT) {
        total_ticks += PCNT_HIGH_LIMIT;
    } else if (edata->watch_point_value == PCNT_LOW_LIMIT) {
        total_ticks += PCNT_LOW_LIMIT;
    }
    return false;
}

//Inicializa la unidad PCNT en modo 4x (canales A/B cruzados, edge+level)
static void encoder_init(void)
{
    //Unidad PCNT,se crea la unidad de conteo definiendo sus límites y un filtro
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10000, // encoder mecánico: rebote de contacto > filtro de un encoder óptico
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    //Inicializa los canales
    //Un canal evalúa los flancos de cambio en GPIO_A comparándolos con el nivel lógico de GPIO_B.
    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = GPIO_A,
        .level_gpio_num = GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    //El otro canal hace lo opuesto Evalua B y compara con A
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = GPIO_B,
        .level_gpio_num = GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    //Solo necesitamos los límites: son los que disparan el wrap-around de hardware
    ESP_LOGI(TAG, "add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_HIGH_LIMIT));

    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

// Timer periódico de micro-ROS: lee la posición actual, calcula RPM y publica ambos valores
static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer == NULL) {
        return;
    }

    // Proteger lectura atómica de total_ticks
    portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&spinlock);
    int64_t current_total = total_ticks;
    portEXIT_CRITICAL(&spinlock);

    int raw_count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &raw_count));
    int32_t current_ticks = (int32_t)(current_total + raw_count);

    int64_t now_us = esp_timer_get_time();
    float dt_s = (last_time_us == 0) ? (PUBLISH_PERIOD_MS / 1000.0f) : (now_us - last_time_us) / 1e6f;
    last_time_us = now_us;

    int32_t delta_ticks = current_ticks - last_ticks;
    last_ticks = current_ticks;

    float rpm = (dt_s > 0.0f) ? ((float)delta_ticks / TICKS_PER_REV) / (dt_s / 60.0f) : 0.0f;

    ticks_msg.data = current_ticks;
    rpm_msg.data = rpm;

    RCSOFTCHECK(rcl_publish(&ticks_publisher, &ticks_msg, NULL));
    RCSOFTCHECK(rcl_publish(&rpm_publisher, &rpm_msg, NULL));

    ESP_LOGI(TAG, "ticks=%ld  rpm=%.2f", (long)current_ticks, rpm);
}

// Tarea micro-ROS: crea el nodo, los publishers y el executor, y los mantiene vivos
static void micro_ros_task(void *arg)
{
    while (1) {
        // 1. ESPERA / PING AL AGENTE
        ESP_LOGI(TAG, "Verificando Agente en IP: %s | Puerto: %s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;

        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options));

        rmw_uros_options_set_client_key(esp_random(), rmw_options);
                                             
#endif

        // Intentar conectar con el agente directamente vía support_init
        rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

        if (rc != RCL_RET_OK) {
            ESP_LOGW(TAG, "No se pudo conectar con el Agente (error %d). Reintentando en 2s...", (int)rc);
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue; // Reintentar en el ciclo while
        }

        ESP_LOGI(TAG, "¡Conectado exitosamente al Agente!");

        rcl_node_t node = rcl_get_zero_initialized_node();
        RCCHECK(rclc_node_init_default(&node, "encoder_node", "", &support));
        ESP_LOGI(TAG, "Nodo creado correctamente");

        RCCHECK(rclc_publisher_init_default(
            &ticks_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "encoder_ticks"));

        RCCHECK(rclc_publisher_init_default(
            &rpm_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "encoder_rpm"));

        rcl_timer_t timer = rcl_get_zero_initialized_timer();
        RCCHECK(rclc_timer_init_default2(
            &timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timer_callback, true));

        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
        RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1)));
        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        // Bucle de publicación
        while (1) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Limpieza si sale del bucle
        rcl_publisher_fini(&ticks_publisher, &node);
        rcl_publisher_fini(&rpm_publisher, &node);
        rcl_timer_fini(&timer);
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    
    // DESACTIVAR POWER SAVE DE WI-FI
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi Power Save desactivado (WIFI_PS_NONE)");

    encoder_init();

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);
}
