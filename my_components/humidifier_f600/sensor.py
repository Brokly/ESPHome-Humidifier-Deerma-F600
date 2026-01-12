import logging
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import automation, pins
from esphome.components import sensor, text_sensor, binary_sensor, select, number
from esphome.automation import maybe_simple_id
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    CONF_HUMIDITY,
    CONF_NUMBER,
    CONF_CURRENT,
    CONF_DATA,
    CONF_VALUE,
    CONF_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    DEVICE_CLASS_MOISTURE,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_STEP,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_ACCURACY_DECIMALS,
)

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@Brokly"]
DEPENDENCIES = ["sensor"]
AUTO_LOAD = ["binary_sensor", "text_sensor", "output", "select", "number"]

CONF_PIN_LED_SYNC = "disp_sync_pin"
CONF_PIN_READ0 = "disp_read0_pin"
CONF_PIN_READ1 = "disp_read1_pin"
CONF_PIN_SENSOR = "sensor_control_pin"
CONF_PIN_SENSOR2 = "sensor2_control_pin"
CONF_SENSOR2_DISABLE ="sensor2_disable"
CONG_EXT_HUMIDITY = "sensor_humidity_id"
CONF_WATER_TANK = "water_ok"
ICON_WATER_TANK = "mdi:water-opacity"
CONF_CURRENT_STATE = "current_state"
ICON_CURRENT_STATE = "mdi:waves-arrow-up"
CONF_CURRENT_PRESET = "current_preset"
CONF_DEST_HUMIDITY = "humidity_destination"

OPTIONS_PRESETS = [
    "Off",
    "Low",
    "Medium",
    "High",
    "Auto",
]

humidifier_f600_ns = cg.esphome_ns.namespace("humidifier_f600")
HumiF600 = humidifier_f600_ns.class_("HumiF600", sensor.Sensor, cg.Component)
HumiF600PresetSelect = humidifier_f600_ns.class_('HumiF600PresetSelect', select.Select)
HumiF600TargetNumber = humidifier_f600_ns.class_('HumiF600TargetNumber', number.Number, cg.Component)

def output_info(config):
    _LOGGER.info(config)
    return config

CONFIG_SCHEMA  = cv.All(
    sensor.sensor_schema(sensor.Sensor).extend(
        {
            cv.GenerateID(): cv.declare_id(HumiF600),
            # внутренний датчик температуры
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            # датчик воды
            cv.Optional(CONF_WATER_TANK): binary_sensor.binary_sensor_schema(
                icon=ICON_WATER_TANK,
                device_class=DEVICE_CLASS_MOISTURE,
            ),
            # текущий режим работы
            cv.Optional(CONF_CURRENT_STATE): text_sensor.text_sensor_schema(
                icon=ICON_CURRENT_STATE,
            ),
            # установка целевой влажности
            cv.Optional(CONF_DEST_HUMIDITY): number.number_schema(number.Number).extend(
                {
                    cv.GenerateID(): cv.declare_id(HumiF600TargetNumber),
                    cv.Optional(CONF_VALUE, default=45): cv.float_,
                    cv.Optional(CONF_MAX_VALUE, default=100): cv.float_,
                    cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                    cv.Optional(CONF_STEP, default=2): cv.positive_float,
                    cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_PERCENT): str, 
                    cv.Optional(CONF_ACCURACY_DECIMALS, default=1): cv.positive_float,
                }
            ),
            # установка режима работы пользователем
            cv.Required(CONF_CURRENT_PRESET): select.select_schema(select.Select).extend(
                {
                    cv.GenerateID(): cv.declare_id(HumiF600PresetSelect),
                }
            ),
            # внешний сенсор влажности
            cv.Required(CONG_EXT_HUMIDITY): cv.use_id(sensor.Sensor),
            # ноги считывания дисплея
            cv.Required(CONF_PIN_LED_SYNC ): pins.gpio_input_pin_schema,
            cv.Required(CONF_PIN_READ0 ): pins.gpio_input_pin_schema,
            cv.Required(CONF_PIN_READ1 ): pins.gpio_input_pin_schema,
            # ноги нажатия на сенсор
            cv.Required(CONF_PIN_SENSOR ): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SENSOR2_DISABLE ): cv.boolean,
            cv.Optional(CONF_PIN_SENSOR2 ): pins.gpio_output_pin_schema,

        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(cv.COMPONENT_SCHEMA)
)            
   
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # температура
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_main_temp(sens))
    # наличие воды
    if CONF_WATER_TANK in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_WATER_TANK])
        cg.add(var.set_water_sensor(sens))
    # текущий статус работы
    if CONF_CURRENT_STATE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_CURRENT_STATE])
        cg.add(var.set_text_operate(sens))
    #внешний сенсор влажности    
    if CONG_EXT_HUMIDITY in config:
       sens = await cg.get_variable(config[CONG_EXT_HUMIDITY])
       cg.add(var.set_ext_humi_sensor(sens))    
    # нога синхронизации чтения
    if CONF_PIN_LED_SYNC in config:
        pin = await cg.gpio_pin_expression(config[CONF_PIN_LED_SYNC])
        cg.add(var.set_sync_pin(pin))
    # нога чтения первого знакоместа
    if CONF_PIN_READ0 in config:
        pin = await cg.gpio_pin_expression(config[CONF_PIN_READ0])
        cg.add(var.set_read0_pin(pin))
    # нога чтения второго знакоместа
    if CONF_PIN_READ1 in config:
        pin = await cg.gpio_pin_expression(config[CONF_PIN_READ1])
        cg.add(var.set_read1_pin(pin))
    # нога управления сенсорной кнопкой
    if CONF_PIN_SENSOR in config:
        pin = await cg.gpio_pin_expression(config[CONF_PIN_SENSOR])
        cg.add(var.set_sens_pin(pin))
    # нога управления второй сенсорной кнопкой
    if CONF_PIN_SENSOR2 in config:
        if CONF_SENSOR2_DISABLE not in config or config[CONF_SENSOR2_DISABLE] == False:
            pin2 = await cg.gpio_pin_expression(config[CONF_PIN_SENSOR2])
            cg.add(var.set_sens2_pin(pin2))
    # селектор выбора режимов работы
    if CONF_CURRENT_PRESET in config:
        sel = await select.new_select(config[CONF_CURRENT_PRESET], options=OPTIONS_PRESETS)
        cg.add(var.set_mode_select(sel))
    # установка целевой влажности    
    if CONF_DEST_HUMIDITY in config:
        num = await number.new_number(config[CONF_DEST_HUMIDITY], min_value=0, max_value=100, step=2)
        cg.add(var.set_humidity_dest(num))
   
