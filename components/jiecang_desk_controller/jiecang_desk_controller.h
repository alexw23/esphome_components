#pragma once

#include <future>
#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "jiecang_desk_number.h"

#define BUTTON_STEP_UP 0
#define BUTTON_STEP_DOWN 1
#define BUTTON_STOP 2
#define BUTTON_POSITION1 3
#define BUTTON_POSITION2 4
#define BUTTON_POSITION3 5
#define BUTTON_POSITION4 6
#define BUTTON_SAVE_POSITION 7
#define BUTTON_MOVE_UP 8
#define BUTTON_MOVE_DOWN 9

#define NUMBER_HEIGHT 0
#define NUMBER_HEIGHT_PCT 1

namespace esphome {
    namespace jiecang_desk_controller {

        class JiecangDeskController : public PollingComponent, public sensor::Sensor, public uart::UARTDevice {
            private:
                float current_height = 0;
                float limit_min = 0;
                float limit_max = 0;
                float physical_min = NAN;
                float physical_max = NAN;

                bool save_position_mode = false;

                uint32_t last_message_time_ = 0;

                float byte2float(int high, int low);
                bool bufferMessage(int data, unsigned int *buffer, int len);
                void handleMessage(unsigned int *message);

            public:
                void update() override;

                void set_sensor_height(sensor::Sensor *sensor) { this->height = sensor; }
                void set_sensor_height_min(sensor::Sensor *sensor) { this->height_min = sensor; }
                void set_sensor_height_max(sensor::Sensor *sensor) { this->height_max = sensor; }
                void set_sensor_height_pct(sensor::Sensor *sensor) { this->height_pct = sensor; }

                void set_sensor_physical_limit_min(sensor::Sensor *sensor) { this->sensor_physical_min = sensor; }
                void set_sensor_physical_limit_max(sensor::Sensor *sensor) { this->sensor_physical_max = sensor; }

                void set_sensor_custom_limit_min(sensor::Sensor *sensor) { this->sensor_custom_limit_min = sensor; }
                void set_sensor_custom_limit_max(sensor::Sensor *sensor) { this->sensor_custom_limit_max = sensor; }
                void set_sensor_has_custom_limit(binary_sensor::BinarySensor *sensor) { this->has_custom_limit = sensor; }

                void set_sensor_position1(sensor::Sensor *sensor) { this->position1 = sensor; }
                void set_sensor_position2(sensor::Sensor *sensor) { this->position2 = sensor; }
                void set_sensor_position3(sensor::Sensor *sensor) { this->position3 = sensor; }
                void set_sensor_position4(sensor::Sensor *sensor) { this->position4 = sensor; }

                void send_simple_command(unsigned char cmd);
                void write_command(unsigned char cmd);
                void add_button(button::Button *btn, int action);
                void add_number(JiecangDeskNumber *number, int type);

                void publish_active_limits();
                void step_up();
                void step_down();
                void stop();
                void goto_position(int pos);
                void save_position(int pos);
                void goto_height(float height);
                void move_up();
                void move_down();

                void request_physical_limits();
                void request_limits();
                void request_settings();

                void number_control(int type, float value);

            protected:
                Sensor *height{nullptr};
                Sensor *height_min{nullptr};
                Sensor *height_max{nullptr};
                Sensor *height_pct{nullptr};

                Sensor *position1{nullptr};
                Sensor *position2{nullptr};
                Sensor *position3{nullptr};
                Sensor *position4{nullptr};

                Sensor *sensor_physical_min{nullptr};
                Sensor *sensor_physical_max{nullptr};

                Sensor *sensor_custom_limit_min{nullptr};
                Sensor *sensor_custom_limit_max{nullptr};
                binary_sensor::BinarySensor *has_custom_limit{nullptr};

                JiecangDeskNumber *number_height{nullptr};
                JiecangDeskNumber *number_height_pct{nullptr};

                void button_press_action(int type);
        };

        class JiecangDeskButton : public Component, public button::Button {
            protected:
                void press_action() override;
        };

    } //namespace jiecang_desk_controller
} //namespace esphome
