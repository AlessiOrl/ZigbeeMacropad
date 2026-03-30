// Custom converter for ESP32-C6 MACROPAD (Z2M 2.x / zigbee-herdsman-converters v5)
// Place in zigbee2mqtt/external_converters/ and restart Z2M.

import {presets as e, access as ea} from 'zigbee-herdsman-converters/lib/exposes';
import {deviceAddCustomCluster} from 'zigbee-herdsman-converters/lib/modernExtend';
import {Zcl} from 'zigbee-herdsman';

const MACROPAD_CLUSTER = 'macropadCluster';
const MACROPAD_CLUSTER_ID = 0xFC00;
const ATTR_DEEP_SLEEP = 0x0001;
const ATTR_DOUBLE_CLICK = 0x0002;
const ATTR_HOLD_PRESS = 0x0003;
const ATTR_ENC_REPORT_INTERVAL = 0x0004;

const RANGES = {
    deep_sleep_timeout: {min: 0, max: 86400},
    double_click_ms: {min: 150, max: 500},
    hold_press_ms: {min: 800, max: 2000},
    enc_report_interval_ms: {min: 20, max: 90},
};

function validateRange(key, value) {
    const numeric = Number(value);
    const range = RANGES[key];

    if (!Number.isFinite(numeric) || !Number.isInteger(numeric)) {
        throw new Error(`${key} must be an integer`);
    }

    if (numeric < range.min || numeric > range.max) {
        throw new Error(`${key} must be between ${range.min} and ${range.max}`);
    }

    return numeric;
}

const fzLocal = {
    macropad_config: {
        cluster: MACROPAD_CLUSTER,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const payload = {};

            const deepSleepTimeout = msg.data?.['deepSleepTimeout'] ?? msg.data?.[ATTR_DEEP_SLEEP] ?? msg.data?.['1'];
            const doubleClickMs = msg.data?.['doubleClickMs'] ?? msg.data?.[ATTR_DOUBLE_CLICK] ?? msg.data?.['2'];
            const holdPressMs = msg.data?.['holdPressMs'] ?? msg.data?.[ATTR_HOLD_PRESS] ?? msg.data?.['3'];
            const encReportIntervalMs = msg.data?.['encReportIntervalMs'] ?? msg.data?.[ATTR_ENC_REPORT_INTERVAL] ?? msg.data?.['4'];

            if (deepSleepTimeout !== undefined) {
                payload.deep_sleep_timeout = deepSleepTimeout;
            }
            if (doubleClickMs !== undefined) {
                payload.double_click_ms = doubleClickMs;
            }
            if (holdPressMs !== undefined) {
                payload.hold_press_ms = holdPressMs;
            }
            if (encReportIntervalMs !== undefined) {
                payload.enc_report_interval_ms = encReportIntervalMs;
            }

            if (Object.keys(payload).length > 0) {
                return payload;
            }
        },
    },
    macropad_button_event: {
        cluster: MACROPAD_CLUSTER,
        type: ['raw'],
        convert: (model, msg, publish, options, meta) => {
            let raw = msg.data;

            if (raw && raw.data && Array.isArray(raw.data)) {
                raw = raw.data;
            } else if (Buffer.isBuffer(raw)) {
                raw = Array.from(raw);
            }

            if (!Array.isArray(raw) || raw.length < 3) {
                meta.logger.debug(
                    `MACROPAD: raw converter got unsupported msg.data = ${JSON.stringify(msg.data)}`
                );
                return {};
            }

            const cmdId = raw[2] ?? 0;

            if (cmdId === 0) {
                const buttonId = raw[3];
                const actionType = raw[4] ?? 0;

                let actionStr = 'default';
                if (actionType === 1) actionStr = 'single';
                else if (actionType === 2) actionStr = 'double';
                else if (actionType === 3) actionStr = 'hold';

                const action = `button_${buttonId}_${actionStr}`;

                meta.logger.info(
                    `MACROPAD: decoded raw=${JSON.stringify(raw)} -> action=${action}`
                );

                return {
                    action,
                    button: buttonId,
                    action_type: actionStr,
                };
            }

            if (cmdId === 1) {
                const direction = raw[3] ?? 0;
                const steps = raw[4] ?? 1;
                const action = (direction === 1) ? 'encoder_right' : 'encoder_left';

                meta.logger.info(
                    `MACROPAD: decoded raw=${JSON.stringify(raw)} -> action=${action} steps=${steps}`
                );

                return {
                    action,
                    steps,
                    direction,
                };
            }

            meta.logger.debug(
                `MACROPAD: unknown cmdId=${cmdId} raw=${JSON.stringify(raw)}`
            );
            return {};
        },
    },
};

const tzLocal = {
    macropad_config: {
        key: ['deep_sleep_timeout', 'double_click_ms', 'hold_press_ms', 'enc_report_interval_ms'],
        convertSet: async (entity, key, value, meta) => {
            const attrMap = {
                deep_sleep_timeout: ATTR_DEEP_SLEEP,
                double_click_ms: ATTR_DOUBLE_CLICK,
                hold_press_ms: ATTR_HOLD_PRESS,
                enc_report_interval_ms: ATTR_ENC_REPORT_INTERVAL,
            };
            const numeric = validateRange(key, value);

            await entity.write(MACROPAD_CLUSTER, {[attrMap[key]]: {value: numeric, type: Zcl.DataType.UINT32}});
            return { state: {[key]: numeric} };
        },
        convertGet: async (entity, key, meta) => {
            const attrMap = {
                deep_sleep_timeout: ATTR_DEEP_SLEEP,
                double_click_ms: ATTR_DOUBLE_CLICK,
                hold_press_ms: ATTR_HOLD_PRESS,
                enc_report_interval_ms: ATTR_ENC_REPORT_INTERVAL,
            };

            await entity.read(MACROPAD_CLUSTER, [attrMap[key]]);
        },
    },
};

export default {
    zigbeeModel: ['MACROPAD'],
    model: 'MACROPAD',
    vendor: 'ORLANDOSLAB',
    description: 'Custom 16-button macropad + encoder (ESP32-C6)',
    extend: [
        // Register cluster ID only; leave attributes and commands empty so that:
        // - Button/encoder frames arrive as 'raw' (not parsed into typed commands)
        // - Attribute writes use numeric IDs (bypasses herdsman's writable check)
        deviceAddCustomCluster(MACROPAD_CLUSTER, {
            ID: MACROPAD_CLUSTER_ID,
            attributes: {},
            commands: {},
            commandsResponse: {},
        }),
    ],
    fromZigbee: [
        fzLocal.macropad_button_event,
        fzLocal.macropad_config,
    ],
    toZigbee: [
        tzLocal.macropad_config,
    ],
    exposes: [
        e.numeric('deep_sleep_timeout', ea.ALL).withDescription('Deep sleep timeout in seconds (0 = disabled)').withValueMin(0).withValueMax(86400),
        e.numeric('double_click_ms', ea.ALL).withDescription('Double click window in milliseconds').withValueMin(150).withValueMax(500),
        e.numeric('hold_press_ms', ea.ALL).withDescription('Hold press threshold in milliseconds').withValueMin(800).withValueMax(2000),
        e.numeric('enc_report_interval_ms', ea.ALL).withDescription('Encoder report interval in milliseconds').withValueMin(20).withValueMax(90),
        e.action([
            'button_0_single', 'button_1_single', 'button_2_single', 'button_3_single',
            'button_4_single', 'button_5_single', 'button_6_single', 'button_7_single',
            'button_8_single', 'button_9_single', 'button_10_single', 'button_11_single',
            'button_12_single', 'button_13_single', 'button_14_single', 'button_15_single',
            'button_0_double', 'button_1_double', 'button_2_double', 'button_3_double',
            'button_4_double', 'button_5_double', 'button_6_double', 'button_7_double',
            'button_8_double', 'button_9_double', 'button_10_double', 'button_11_double',
            'button_12_double', 'button_13_double', 'button_14_double', 'button_15_double',
            'button_0_hold', 'button_1_hold', 'button_2_hold', 'button_3_hold',
            'button_4_hold', 'button_5_hold', 'button_6_hold', 'button_7_hold',
            'button_8_hold', 'button_9_hold', 'button_10_hold', 'button_11_hold',
            'button_12_hold', 'button_13_hold', 'button_14_hold', 'button_15_hold',
            'encoder_left', 'encoder_right',
        ]),
    ],
};