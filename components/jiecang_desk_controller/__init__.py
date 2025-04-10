import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart, sensor, button, number, binary_sensor
from esphome.const import CONF_ID, CONF_HEIGHT, CONF_UNIT_OF_MEASUREMENT, CONF_ACCURACY_DECIMALS, UNIT_CENTIMETER, UNIT_PERCENT

DEPENDENCIES = ['uart', 'binary_sensor']
AUTO_LOAD = ['sensor', 'button', 'number', 'binary_sensor']

jiecang_desk_controller_ns = cg.esphome_ns.namespace('jiecang_desk_controller')

JiecangDeskController = jiecang_desk_controller_ns.class_('JiecangDeskController', cg.Component, uart.UARTDevice)
JiecangDeskButton = jiecang_desk_controller_ns.class_('JiecangDeskButton', button.Button, cg.Component)
JiecangDeskNumber = jiecang_desk_controller_ns.class_('JiecangDeskNumber', number.Number, cg.Component)


CONF_SENSORS = "sensors"
CONF_BUTTONS = "buttons"
CONF_NUMBERS = "numbers"
CONF_UNIT = "unit"
CONF_STEP_UP = "step_up"
CONF_STEP_DOWN = "step_down"
CONF_STOP = "stop"
CONF_HEIGHT_MIN = "height_min"
CONF_HEIGHT_MAX = "height_max"
CONF_HEIGHT_PCT = "height_pct"
CONF_POSITION1 = "position1"
CONF_POSITION2 = "position2"
CONF_POSITION3 = "position3"
CONF_POSITION4 = "position4"
CONF_SAVE_POSITION = "save_position"
CONF_MOVE_UP = "move_up"
CONF_MOVE_DOWN = "move_down"
CONF_CUSTOM_LIMIT_MIN = "custom_limit_min"
CONF_CUSTOM_LIMIT_MAX = "custom_limit_max"
CONF_PHYSICAL_LIMIT_MIN = "physical_limit_min"
CONF_PHYSICAL_LIMIT_MAX = "physical_limit_max"
CONF_LIMIT_MIN = "limit_min"
CONF_LIMIT_MAX = "limit_max"
CONF_HAS_CUSTOM_LIMIT = "has_custom_limit"

button_constants = {}
button_constants[CONF_STEP_UP] = 0
button_constants[CONF_STEP_DOWN] = 1
button_constants[CONF_STOP] = 2
button_constants[CONF_POSITION1] = 3
button_constants[CONF_POSITION2] = 4
button_constants[CONF_POSITION3] = 5
button_constants[CONF_POSITION4] = 6
button_constants[CONF_SAVE_POSITION] = 7
button_constants[CONF_MOVE_UP] = 8
button_constants[CONF_MOVE_DOWN] = 9

# Backward compatibility
CONF_RAISE = "raise"
CONF_LOWER = "lower"
button_constants[CONF_RAISE] = button_constants[CONF_STEP_UP]
button_constants[CONF_LOWER] = button_constants[CONF_STEP_DOWN]

number_constants = {}
number_constants[CONF_HEIGHT] = 0
number_constants[CONF_HEIGHT_PCT] = 1
number_constants[CONF_PHYSICAL_LIMIT_MIN] = 2
number_constants[CONF_PHYSICAL_LIMIT_MAX] = 3
number_constants[CONF_CUSTOM_LIMIT_MIN] = 4
number_constants[CONF_CUSTOM_LIMIT_MAX] = 5
number_constants[CONF_LIMIT_MIN] = 6
number_constants[CONF_LIMIT_MAX] = 7
number_constants[CONF_HAS_CUSTOM_LIMIT] = 8

CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(JiecangDeskController),
    cv.Optional(CONF_SENSORS): cv.Schema({
        cv.Optional(CONF_HEIGHT): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_UNIT): sensor.sensor_schema(
            accuracy_decimals = 0
        ),
        cv.Optional(CONF_HEIGHT_PCT): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_PERCENT
        ),
        cv.Optional(CONF_HEIGHT_MIN): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_HEIGHT_MAX): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_POSITION1): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_POSITION2): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_POSITION3): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_POSITION4): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_PHYSICAL_LIMIT_MIN): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_PHYSICAL_LIMIT_MAX): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_CUSTOM_LIMIT_MIN): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_CUSTOM_LIMIT_MAX): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_LIMIT_MIN): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
        cv.Optional(CONF_LIMIT_MAX): sensor.sensor_schema(
            accuracy_decimals = 1,
            unit_of_measurement = UNIT_CENTIMETER
        ),
    }),
    cv.Optional(CONF_HAS_CUSTOM_LIMIT): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_NUMBERS): cv.Schema({
        cv.Optional(CONF_HEIGHT): number.NUMBER_SCHEMA.extend({
            cv.GenerateID(): cv.declare_id(JiecangDeskNumber),
            cv.Optional(CONF_ACCURACY_DECIMALS, default=1): cv.int_,
            cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CENTIMETER): cv.string,
        }),
        cv.Optional(CONF_HEIGHT_PCT): number.NUMBER_SCHEMA.extend({
            cv.GenerateID(): cv.declare_id(JiecangDeskNumber),
            cv.Optional(CONF_ACCURACY_DECIMALS, default=1): cv.int_,
            cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_PERCENT): cv.string,
        }),
    }),
    cv.Optional(CONF_BUTTONS): cv.Schema({
        cv.Optional(CONF_STEP_UP): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_STEP_DOWN): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_STOP): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_POSITION1): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_POSITION2): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_POSITION3): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_POSITION4): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_SAVE_POSITION): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_MOVE_UP): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_MOVE_DOWN): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        # Backward compatibility
        cv.Optional(CONF_RAISE): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
        cv.Optional(CONF_LOWER): button.BUTTON_SCHEMA.extend({cv.GenerateID(): cv.declare_id(JiecangDeskButton)}),
    }),
}).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_SENSORS in config:
        sensors = config[CONF_SENSORS]

        if CONF_HEIGHT in sensors:
            sens = await sensor.new_sensor(sensors[CONF_HEIGHT])
            cg.add(var.set_sensor_height(sens))
        if CONF_UNIT in sensors:
            sens = await sensor.new_sensor(sensors[CONF_UNIT])
            cg.add(var.set_sensor_unit(sens))
        if CONF_HEIGHT_MIN in sensors:
            sens = await sensor.new_sensor(sensors[CONF_HEIGHT_MIN])
            cg.add(var.set_sensor_height_min(sens))
        if CONF_HEIGHT_MAX in sensors:
            sens = await sensor.new_sensor(sensors[CONF_HEIGHT_MAX])
            cg.add(var.set_sensor_height_max(sens))
        if CONF_HEIGHT_PCT in sensors:
            sens = await sensor.new_sensor(sensors[CONF_HEIGHT_PCT])
            cg.add(var.set_sensor_height_pct(sens))
        if CONF_POSITION1 in sensors:
            sens = await sensor.new_sensor(sensors[CONF_POSITION1])
            cg.add(var.set_sensor_position1(sens))
        if CONF_POSITION2 in sensors:
            sens = await sensor.new_sensor(sensors[CONF_POSITION2])
            cg.add(var.set_sensor_position2(sens))
        if CONF_POSITION3 in sensors:
            sens = await sensor.new_sensor(sensors[CONF_POSITION3])
            cg.add(var.set_sensor_position3(sens))
        if CONF_POSITION4 in sensors:
            sens = await sensor.new_sensor(sensors[CONF_POSITION4])
            cg.add(var.set_sensor_position4(sens))
        if CONF_CUSTOM_LIMIT_MIN in sensors:
            sens = await sensor.new_sensor(sensors[CONF_CUSTOM_LIMIT_MIN])
            cg.add(var.set_sensor_custom_limit_min(sens))
        if CONF_CUSTOM_LIMIT_MAX in sensors:
            sens = await sensor.new_sensor(sensors[CONF_CUSTOM_LIMIT_MAX])
            cg.add(var.set_sensor_custom_limit_max(sens))
        if CONF_PHYSICAL_LIMIT_MIN in sensors:
            sens = await sensor.new_sensor(sensors[CONF_PHYSICAL_LIMIT_MIN])
            cg.add(var.set_sensor_physical_limit_min(sens))
        if CONF_PHYSICAL_LIMIT_MAX in sensors:
            sens = await sensor.new_sensor(sensors[CONF_PHYSICAL_LIMIT_MAX])
            cg.add(var.set_sensor_physical_limit_max(sens))
        if CONF_LIMIT_MIN in sensors:
            sens = await sensor.new_sensor(sensors[CONF_LIMIT_MIN])
            cg.add(var.set_sensor_limit_min(sens))
        if CONF_LIMIT_MAX in sensors:
            sens = await sensor.new_sensor(sensors[CONF_LIMIT_MAX])
            cg.add(var.set_sensor_limit_max(sens))

    if CONF_HAS_CUSTOM_LIMIT in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_HAS_CUSTOM_LIMIT])
        cg.add(var.set_sensor_has_custom_limit(bs))

    if CONF_BUTTONS in config:
        buttons = config[CONF_BUTTONS]
        for button_type in buttons.keys():
            btn = await button.new_button(buttons[button_type])
            cg.add(var.add_button(btn, button_constants[button_type]))

    if CONF_NUMBERS in config:
        numbers = config[CONF_NUMBERS]
        for number_type in numbers.keys():
            num = await number.new_number(numbers[number_type], min_value=0, max_value=100, step=.1)
            cg.add(var.add_number(num, number_constants[number_type]))

