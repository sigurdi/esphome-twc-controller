"""TWC Gen2 RS485 fake-master component (rewrite of jnicolson/esphome-twc-controller, multi-charger).

Top level: bus, grid_type, max_current (main fuse, shared by all chargers) and the entity names, written once.
`multi_charger:` (optional): the TWCs on the bus (1-4, each its own HA device) and the shared control:
control mode (Current / Power), max power, priority, total current, total power.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import (
    binary_sensor,
    number,
    select,
    sensor,
    switch,
    text_sensor,
    uart,
)
from esphome.const import (
    CONF_CURRENT,
    CONF_DEVICE_ID,
    CONF_FLOW_CONTROL_PIN,
    CONF_ID,
    CONF_NAME,
    CONF_POWER,
    CONF_STATE,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_RUNNING,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_KILOWATT_HOURS,
    UNIT_VOLT,
    UNIT_WATT,
)

CODEOWNERS = []
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor", "number", "switch", "select"]

CONF_TWCID = "twc_id"
CONF_MIN_CURRENT = "min_current"
CONF_MAX_CURRENT = "max_current"
CONF_SET_CURRENT = "set_current"
CONF_NOMINAL_VOLTAGE = "nominal_voltage"
CONF_PHASE_THRESHOLD = "phase_threshold"
CONF_STOP_REASSERT = "stop_reassert_interval"

CONF_CURRENT_LIMIT = "current_limit"
CONF_CHARGING = "charging"
CONF_GRID_TYPE = "grid_type"

CONF_MAX_ALLOWABLE_CURRENT = "max_allowable_current"
CONF_TOTAL_KWH = "total_kwh_delivered"
CONF_ACTUAL_CURRENT = "actual_current"
CONF_OFFERED_CURRENT = "offered_current"
CONF_ACTIVE_PHASES = "active_phases"
CONF_SERIAL = "serial"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_VIN = "connected_vin"
CONF_STATE_TEXT = "state_text"
CONF_CHARGE_MODE = "charge_mode"
CONF_ONLINE = "online"
CONF_VEHICLE_CHARGING = "vehicle_charging"
CONF_SESSION_ENERGY = "session_energy"
CONF_CAR_INFO = "car_info"

CONF_CHARGERS = "chargers"
CONF_SLAVE_ID = "slave_id"
CONF_TOTAL_CURRENT = "total_current"
CONF_ASSIGNED_CURRENT = "assigned_current"
CONF_MAX_POWER = "max_power"
CONF_MAX_POWER_LIMIT = "max_power_limit"
CONF_PRIORITY = "priority"
CONF_TOTAL_POWER = "total_power"
CONF_CONTROL_MODE = "control_mode"
CONTROL_MODES = ["Current", "Power"]  # index = mode_ in the .h
CONF_CAR_TYPE = "car_type"
CAR_TYPES = ["Auto", "1-phase", "3-phase", "2 sub-chargers"]  # index = CAR_TYPE_OPTIONS in the .cpp
CONF_CIRCUIT_POWER = "circuit_power"
CONF_TOTAL_ENERGY = "total_energy"
CONF_ENERGY_OFFSET = "energy_offset"
MAX_CHARGERS = 4

ICON_CURRENT_AC = "mdi:current-ac"

twc_ns = cg.esphome_ns.namespace("twc_controller")
TWCController = twc_ns.class_("TWCController", cg.Component, uart.UARTDevice)
TWCCharger = twc_ns.class_("TWCCharger")
TWCCurrentNumber = twc_ns.class_("TWCCurrentNumber", number.Number)
TWCSettingNumber = twc_ns.class_("TWCSettingNumber", number.Number)
TWCModeSelect = twc_ns.class_("TWCModeSelect", select.Select)
TWCCarTypeSelect = twc_ns.class_("TWCCarTypeSelect", select.Select)
TWCOffsetNumber = twc_ns.class_("TWCOffsetNumber", number.Number)
SET_TOTAL_AMPS, SET_MAX_POWER, SET_PRIORITY = 0, 1, 2  # Setting enum in the .h
TWCChargingSwitch = twc_ns.class_("TWCChargingSwitch", switch.Switch)
# Installation setting; index order matches GridType in the .h
GRID_TYPES = {"SINGLE_PHASE": 0, "TN_3PHASE": 1, "IT_3PHASE": 2}


def _amp_sensor(decimals=1):
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        icon=ICON_CURRENT_AC,
        accuracy_decimals=decimals,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _volt_sensor():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )


SENSORS = {
    CONF_CURRENT: _amp_sensor(),
    CONF_ACTUAL_CURRENT: _amp_sensor(),
    CONF_OFFERED_CURRENT: _amp_sensor(),
    CONF_MAX_ALLOWABLE_CURRENT: _amp_sensor(0),
    CONF_ASSIGNED_CURRENT: _amp_sensor(0),  # limit after the total split, 0 = paused
    "phase_1_current": _amp_sensor(),
    "phase_2_current": _amp_sensor(),
    "phase_3_current": _amp_sensor(),
    "phase_1_voltage": _volt_sensor(),
    "phase_2_voltage": _volt_sensor(),
    "phase_3_voltage": _volt_sensor(),
    CONF_TOTAL_KWH: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    CONF_POWER: sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    # Since plug-in: TWC counter steps (1 kWh), or own power integration for 2 sub-chargers.
    CONF_SESSION_ENERGY: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    # Own lifetime counter + offset, saved in flash. IT 3-phase: from our power; else TWC counter steps.
    CONF_TOTAL_ENERGY: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    CONF_ACTIVE_PHASES: sensor.sensor_schema(
        icon="mdi:sine-wave", accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT
    ),
    CONF_STATE: sensor.sensor_schema(
        icon="mdi:state-machine",
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
}

TEXT_SENSORS = {
    CONF_SERIAL: text_sensor.text_sensor_schema(
        icon="mdi:identifier", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    CONF_FIRMWARE_VERSION: text_sensor.text_sensor_schema(
        icon="mdi:numeric", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ),
    CONF_VIN: text_sensor.text_sensor_schema(icon="mdi:car"),
    CONF_STATE_TEXT: text_sensor.text_sensor_schema(icon="mdi:ev-station"),
    CONF_CHARGE_MODE: text_sensor.text_sensor_schema(icon="mdi:sine-wave"),
    CONF_CAR_INFO: text_sensor.text_sensor_schema(icon="mdi:car-info"),
    "plug_state": text_sensor.text_sensor_schema(icon="mdi:ev-plug-type2"),
    "model": text_sensor.text_sensor_schema(icon="mdi:ev-station", entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
}

BINARY_SENSORS = {
    CONF_ONLINE: binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_CONNECTIVITY,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    CONF_VEHICLE_CHARGING: binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING, icon="mdi:ev-plug-type2"
    ),
    # On while the quick car type check runs (IT 3-phase, no VIN): let HA current automations wait.
    "type_check": binary_sensor.binary_sensor_schema(icon="mdi:car-search"),
}

_CURRENT_NUMBER = number.number_schema(
    TWCCurrentNumber,
    icon=ICON_CURRENT_AC,
    unit_of_measurement=UNIT_AMPERE,
    device_class=DEVICE_CLASS_CURRENT,
)
_CHARGING_SWITCH = switch.switch_schema(
    TWCChargingSwitch,
    icon="mdi:ev-station",
    default_restore_mode="RESTORE_DEFAULT_ON",
)
_ENTITIES = {
    **{cv.Optional(k): v for k, v in SENSORS.items()},
    **{cv.Optional(k): v for k, v in TEXT_SENSORS.items()},
    **{cv.Optional(k): v for k, v in BINARY_SENSORS.items()},
}

CONF_MULTI = "multi_charger"
CONF_CHARGER_LIST = "charger_list"  # internal: one full entity set per charger, built by _expand_chargers

_ENTITY_KEYS = [*SENSORS, *TEXT_SENSORS, *BINARY_SENSORS, CONF_CURRENT_LIMIT, CONF_CHARGING, CONF_CAR_TYPE, CONF_ENERGY_OFFSET]
_ENTRY_KEYS = {CONF_DEVICE_ID, CONF_SLAVE_ID, CONF_SET_CURRENT}

# One charger (TWC). Built from the top-level entity keys, never written by hand.
CHARGER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TWCCharger),
        cv.Optional(CONF_SLAVE_ID): cv.hex_int_range(0x0001, 0xFFFE),
        cv.Optional(CONF_SET_CURRENT): cv.int_range(min=6, max=80),
        cv.Optional(CONF_CURRENT_LIMIT): _CURRENT_NUMBER,
        cv.Optional(CONF_CHARGING): _CHARGING_SWITCH,
        # Car type (IT 3-phase): Auto, or force 1-phase / 3-phase / 2 sub-chargers for this charger's car.
        cv.Optional(CONF_CAR_TYPE): select.select_schema(TWCCarTypeSelect, icon="mdi:car-cog"),
        # kWh added to total_energy, e.g. to start at the value the charger had before.
        cv.Optional(CONF_ENERGY_OFFSET): number.number_schema(
            TWCOffsetNumber, icon="mdi:counter", unit_of_measurement=UNIT_KILOWATT_HOURS
        ),
        **_ENTITIES,
    }
)

# Shared control (several chargers on one fuse, and/or power control). Lives on the main device.
MULTI_SCHEMA = cv.Schema(
    {
        # Control mode: Current (Set Current rules) or Power (Max Power rules). Without it: Power when Max Power > 0.
        cv.Optional(CONF_CONTROL_MODE): select.select_schema(TWCModeSelect, icon="mdi:tune"),
        # W for all cars together. Range: what max_current gives on this grid (see _max_power).
        cv.Optional(CONF_MAX_POWER): number.number_schema(
            TWCSettingNumber, icon="mdi:flash", unit_of_measurement=UNIT_WATT, device_class=DEVICE_CLASS_POWER
        ),
        cv.Optional(CONF_MAX_POWER_LIMIT): cv.int_range(min=1000, max=100000),
        # % of the power for charger 1; the others share the rest equally. 50 = equal for 2.
        cv.Optional(CONF_PRIORITY): number.number_schema(
            TWCSettingNumber, icon="mdi:scale-balance", unit_of_measurement="%"
        ),
        # Lower the shared limit at runtime (e.g. solar). Range min_current..max_current.
        cv.Optional(CONF_TOTAL_CURRENT): number.number_schema(
            TWCSettingNumber, icon=ICON_CURRENT_AC, unit_of_measurement=UNIT_AMPERE, device_class=DEVICE_CLASS_CURRENT
        ),
        # Power of the circuit the chargers are on (a homeassistant sensor). Optional: used on IT 3-phase to
        # tell 1-phase from 3-phase cars without a useful VIN, from the power jump when the current changes.
        cv.Optional(CONF_CIRCUIT_POWER): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_TOTAL_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # The TWCs on the bus, in priority order. Each gets the top-level entities on its own HA device.
        cv.Optional(CONF_CHARGERS): cv.All(
            cv.ensure_list(
                cv.Schema(
                    {
                        cv.Optional(CONF_DEVICE_ID): cv.sub_device_id,
                        cv.Optional(CONF_SLAVE_ID): cv.hex_int_range(0x0001, 0xFFFE),
                        cv.Optional(CONF_SET_CURRENT): cv.int_range(min=6, max=80),
                    }
                )
            ),
            cv.Length(min=1, max=MAX_CHARGERS),
        ),
    }
)


def _expand_chargers(config):
    """Top-level entity keys are written once and copied to every charger in multi_charger.chargers,
    each with that charger's device_id. The last of several chargers gets no Set Current: it takes the rest."""
    if not isinstance(config, dict):
        return config
    config = dict(config)
    mc = config.get(CONF_MULTI) or {}
    entries = mc.get(CONF_CHARGERS) if isinstance(mc, dict) else None
    if not isinstance(entries, list) or not entries:
        entries = [{}]
    template = {k: config.pop(k) for k in _ENTITY_KEYS if k in config}
    template.setdefault(CONF_CURRENT_LIMIT, {CONF_NAME: "Set Current"})
    template.setdefault(CONF_CHARGING, {CONF_NAME: "Charging Enabled"})
    out = []
    for idx, entry in enumerate(entries):
        entry = entry if isinstance(entry, dict) else {}
        bad = set(entry) - _ENTRY_KEYS
        if bad:
            raise cv.Invalid(f"{CONF_MULTI}.{CONF_CHARGERS}: unknown key(s) {sorted(bad)}; allowed {sorted(_ENTRY_KEYS)}")
        ch = {}
        for key, ent in template.items():
            if not isinstance(ent, dict):
                ent = {CONF_NAME: ent}  # short form "power: Charging Power"
            ent = dict(ent)
            if idx > 0:
                ent.pop(CONF_ID, None)  # ids must be unique; copies get generated ones
            if entry.get(CONF_DEVICE_ID) and CONF_DEVICE_ID not in ent:
                ent[CONF_DEVICE_ID] = entry[CONF_DEVICE_ID]
            ch[key] = ent
        if len(entries) > 1 and idx == len(entries) - 1:
            ch.pop(CONF_CURRENT_LIMIT, None)
        for key in (CONF_SLAVE_ID, CONF_SET_CURRENT):
            if key in entry:
                ch[key] = entry[key]
        out.append(ch)
    config[CONF_CHARGER_LIST] = out
    return config


def _validate(config):
    if config[CONF_MAX_CURRENT] <= config[CONF_MIN_CURRENT]:
        raise cv.Invalid(f"{CONF_MAX_CURRENT} must be greater than {CONF_MIN_CURRENT}")
    chargers = config[CONF_CHARGER_LIST]
    for c in [config] + chargers:
        sc = c.get(CONF_SET_CURRENT)
        if sc is not None and not config[CONF_MIN_CURRENT] <= sc <= config[CONF_MAX_CURRENT]:
            raise cv.Invalid(f"{CONF_SET_CURRENT} must be between {CONF_MIN_CURRENT} and {CONF_MAX_CURRENT}")
    ids = [c[CONF_SLAVE_ID] for c in chargers if CONF_SLAVE_ID in c]
    if len(ids) != len(set(ids)):
        raise cv.Invalid(f"{CONF_SLAVE_ID} must be unique")
    entries = config.get(CONF_MULTI, {}).get(CONF_CHARGERS, [])
    if len(entries) > 1 and sum(1 for e in entries if not e.get(CONF_DEVICE_ID)) > 1:
        raise cv.Invalid(f"{CONF_MULTI}.{CONF_CHARGERS}: give every charger a device_id (one may stay on the main device)")
    return config


CONFIG_SCHEMA = cv.All(
    _expand_chargers,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TWCController),
            cv.Required(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_TWCID, default=0x7777): cv.hex_int_range(0x0001, 0xFFFE),
            cv.Required(CONF_GRID_TYPE): cv.enum(GRID_TYPES, upper=True),
            cv.Optional(CONF_MIN_CURRENT, default=6): cv.int_range(min=6, max=80),
            # Main fuse / supply cable: the most all chargers together may draw (one charger can use all).
            cv.Optional(CONF_MAX_CURRENT, default=16): cv.int_range(min=6, max=80),
            cv.Optional(CONF_SET_CURRENT): cv.int_range(min=6, max=80),
            cv.Optional(CONF_NOMINAL_VOLTAGE, default=230.0): cv.float_range(min=100, max=480),
            cv.Optional(CONF_PHASE_THRESHOLD, default=1.0): cv.float_range(min=0.5, max=6),
            cv.Optional(CONF_STOP_REASSERT, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MULTI): MULTI_SCHEMA,
            cv.Required(CONF_CHARGER_LIST): cv.ensure_list(CHARGER_SCHEMA),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    _validate,
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "twc_controller", baud_rate=9600, require_tx=True, require_rx=True
)


async def _charger(ctrl, conf, top):
    ch = cg.new_Pvariable(conf[CONF_ID])
    await cg.register_parented(ch, ctrl)
    cg.add(ctrl.add_charger(ch))
    if CONF_SLAVE_ID in conf:
        cg.add(ch.set_pinned_id(conf[CONF_SLAVE_ID]))
    initial = conf.get(CONF_SET_CURRENT, top.get(CONF_SET_CURRENT, top[CONF_MIN_CURRENT]))
    cg.add(ch.set_initial_current(initial))
    if CONF_CURRENT_LIMIT in conf:
        num = await number.new_number(
            conf[CONF_CURRENT_LIMIT], min_value=top[CONF_MIN_CURRENT], max_value=top[CONF_MAX_CURRENT], step=1
        )
        await cg.register_parented(num, ch)
        cg.add(ch.set_current_number(num))
    if CONF_CHARGING in conf:
        sw = await switch.new_switch(conf[CONF_CHARGING])
        await cg.register_parented(sw, ch)
        cg.add(ch.set_charging_switch(sw))
    if CONF_CAR_TYPE in conf:
        sel = await select.new_select(conf[CONF_CAR_TYPE], options=CAR_TYPES)
        await cg.register_parented(sel, ch)
        cg.add(ch.set_car_type_select(sel))
    if CONF_ENERGY_OFFSET in conf:
        num = await number.new_number(conf[CONF_ENERGY_OFFSET], min_value=-100000, max_value=1000000, step=0.01)
        await cg.register_parented(num, ch)
        cg.add(ch.set_energy_offset_number(num))
    for key in SENSORS:
        if key in conf:
            s = await sensor.new_sensor(conf[key])
            cg.add(getattr(ch, f"set_{key}_sensor")(s))
    for key in TEXT_SENSORS:
        if key in conf:
            s = await text_sensor.new_text_sensor(conf[key])
            cg.add(getattr(ch, f"set_{key}_text_sensor")(s))
    for key in BINARY_SENSORS:
        if key in conf:
            s = await binary_sensor.new_binary_sensor(conf[key])
            cg.add(getattr(ch, f"set_{key}_binary_sensor")(s))


def _max_power(config, mc):
    """Max for "Max Power": what max_current (the shared fuse) gives on this grid, rounded up to 100 W.
    TN 3-phase 3·V·I (32 A -> 22 kW, 16 A -> 11 kW), IT 3-phase √3·V·I (32 A -> 12.7 kW), 1-phase V·I (32 A -> 7.4 kW)."""
    if CONF_MAX_POWER_LIMIT in mc:
        return mc[CONF_MAX_POWER_LIMIT]
    factor = {"SINGLE_PHASE": 1.0, "TN_3PHASE": 3.0, "IT_3PHASE": 3**0.5}[config[CONF_GRID_TYPE]]
    w = factor * config[CONF_NOMINAL_VOLTAGE] * config[CONF_MAX_CURRENT]
    return int(-(-w // 100) * 100)


async def _setting(var, conf, setting, lo, hi, step, initial):
    n = await number.new_number(conf, min_value=lo, max_value=hi, step=step)
    await cg.register_parented(n, var)
    cg.add(n.set_setting(setting))
    cg.add(var.set_setting_number(setting, n, initial))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
    cg.add(var.set_flow_control_pin(pin))
    cg.add(var.set_twcid(config[CONF_TWCID]))
    cg.add(var.set_min_current(config[CONF_MIN_CURRENT]))
    cg.add(var.set_max_current(config[CONF_MAX_CURRENT]))
    cg.add(var.set_nominal_voltage(config[CONF_NOMINAL_VOLTAGE]))
    cg.add(var.set_phase_threshold(config[CONF_PHASE_THRESHOLD]))
    cg.add(var.set_stop_reassert_interval(config[CONF_STOP_REASSERT]))
    cg.add(var.set_grid_type(GRID_TYPES[config[CONF_GRID_TYPE]]))

    mc = config.get(CONF_MULTI, {})
    lo, hi = config[CONF_MIN_CURRENT], config[CONF_MAX_CURRENT]
    if CONF_TOTAL_CURRENT in mc:
        await _setting(var, mc[CONF_TOTAL_CURRENT], SET_TOTAL_AMPS, lo, hi, 1, hi)
    if CONF_MAX_POWER in mc:
        pmax = _max_power(config, mc)
        # With a control mode select the mode decides, so start at full power; without it 0 = off.
        await _setting(var, mc[CONF_MAX_POWER], SET_MAX_POWER, 0, pmax, 100, pmax if CONF_CONTROL_MODE in mc else 0)
    if CONF_PRIORITY in mc:
        await _setting(var, mc[CONF_PRIORITY], SET_PRIORITY, 0, 100, 5, 50)
    if CONF_CONTROL_MODE in mc:
        sel = await select.new_select(mc[CONF_CONTROL_MODE], options=CONTROL_MODES)
        await cg.register_parented(sel, var)
        cg.add(var.set_mode_select(sel))
    if CONF_CIRCUIT_POWER in mc:
        cp = await cg.get_variable(mc[CONF_CIRCUIT_POWER])
        cg.add(var.set_circuit_power_sensor(cp))
    if CONF_TOTAL_POWER in mc:
        s = await sensor.new_sensor(mc[CONF_TOTAL_POWER])
        cg.add(var.set_total_power_sensor(s))

    for conf in config[CONF_CHARGER_LIST]:
        await _charger(var, conf, config)
