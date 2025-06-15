#include <micro_ros_arduino.h>

#include <example_interfaces/msg/u_int8.h>
#include <example_interfaces/msg/u_int64.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <stdio.h>

#include "wifi_info.h"

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
  #error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#define SERIAL_BAUD_RATE 9600

#define LOOP_SLEEP_MS 10

#define NODE_NAMESPACE ""
#define NODE_NAME "traffic_light_micro_node"
#define MICROROS_AGENT_IP "192.168.1.160"
#define MICROROS_AGENT_PORT 8888

#define TOPIC_NAME_HEARTBEAT "traffic_light/heartbeat"
#define TOPIC_NAME_LIGHTS_STATE "traffic_light/lights_state"
#define TOPIC_NAME_SET_LIGHTS "traffic_light/set_lights"

#define PIN_RED 10
#define PIN_YLW 11
#define PIN_GRN 12

#define PIN_LED 13

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t heartbeat_publisher;
example_interfaces__msg__UInt64 heartbeat_msg;

rcl_publisher_t lights_state_publisher;
example_interfaces__msg__UInt8 lights_state_msg;

rcl_subscription_t set_lights_sub;
example_interfaces__msg__UInt8 set_lights_msg;

void soft_error_check(const char* op, const rcl_ret_t rc)
{
  if (rc != RCL_RET_OK)
  {
    Serial.print("Soft error check failed: '");
    Serial.print(op);
    Serial.print("' (error code = ");
    Serial.print(rc);
    Serial.println(")");
  }
}

void strict_error_check(const char* op, const rcl_ret_t rc)
{
  if (rc != RCL_RET_OK) 
  {
    while(1)
    {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(1000);
      Serial.print("STRICT error check failed: '");
      Serial.print(op);
      Serial.print("' (error code = ");
      Serial.print(rc);
      Serial.println(")");
    }
  }
}

bool get_bit(const int value, const int bit_number)
{
  return (value >> bit_number) & 1;
}

void set_lights_callback(const void* msg_in)
{
  lights_state_msg = example_interfaces__msg__UInt8(
    *(static_cast<const example_interfaces__msg__UInt8*>(msg_in))
  );

  const bool red = get_bit(lights_state_msg.data, 0);
  const bool ylw = get_bit(lights_state_msg.data, 1);
  const bool grn = get_bit(lights_state_msg.data, 2);

  digitalWrite(PIN_RED, red ? HIGH : LOW);
  digitalWrite(PIN_YLW, ylw ? HIGH : LOW);
  digitalWrite(PIN_GRN, grn ? HIGH : LOW);

  soft_error_check(
    "publish lights state",
    rcl_publish(&lights_state_publisher, &lights_state_msg, NULL)
  );

  Serial.print("r=");
  Serial.print(static_cast<const char*>(red?"Y":"N"));
  Serial.print(", y=");
  Serial.print(static_cast<const char*>(ylw?"Y":"N"));
  Serial.print(", g=");
  Serial.println(static_cast<const char*>(grn?"Y":"N"));
}

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  set_microros_wifi_transports(
    MICROROS_WIFI_SSID,
    MICROROS_WIFI_PASSWORD,
    MICROROS_AGENT_IP,
    MICROROS_AGENT_PORT
  );

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_YLW, OUTPUT);
  pinMode(PIN_GRN, OUTPUT);
  digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_YLW, LOW);
  digitalWrite(PIN_GRN, LOW);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  delay(5000);

  allocator = rcl_get_default_allocator();

  strict_error_check(
    "init support",
    rclc_support_init(&support, 0, NULL, &allocator)
  );

  strict_error_check(
    "init node",
    rclc_node_init_default(&node, NODE_NAME, NODE_NAMESPACE, &support)
  );

  strict_error_check(
    "init heartbeat publisher",
    rclc_publisher_init_best_effort(
      &heartbeat_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(example_interfaces, msg, UInt64),
      TOPIC_NAME_HEARTBEAT
    )
  );

  strict_error_check(
    "init lights state publisher",
    rclc_publisher_init_best_effort(
      &lights_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(example_interfaces, msg, UInt8),
      TOPIC_NAME_LIGHTS_STATE
    )
  );

  strict_error_check(
    "init set lights subscription",
    rclc_subscription_init_default(
      &set_lights_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(example_interfaces, msg, UInt8),
      TOPIC_NAME_SET_LIGHTS
    )
  );

  strict_error_check(
    "init executor",
    rclc_executor_init(
      &executor,
      &support.context,
      1,
      &allocator
    )
  );
  strict_error_check(
    "add set lights subscription",
    rclc_executor_add_subscription(
      &executor,
      &set_lights_sub,
      &set_lights_msg,
      &set_lights_callback,
      ON_NEW_DATA
    )
  );

  heartbeat_msg.data = 0;

  digitalWrite(PIN_LED, HIGH);
  Serial.println("Finished setup!");
}

void loop()
{
    delay(LOOP_SLEEP_MS);

    strict_error_check(
      "executor spin some",
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(LOOP_SLEEP_MS))
    );

    soft_error_check(
      "publish heartbeat",
      rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL)
    );
    heartbeat_msg.data++;
}
