/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include "DShotRMT.h"
#include <cstring>

static bool isErpmTelemetryFrame(uint16_t raw)
{
    if (raw == 0x0FFFU)
    {
        return false;
    }

    // Per EDT spec, eRPM frames are those with either a zero low nibble or
    // bit 8 set. Other values are extended telemetry frames.
    return ((raw & 0x000FU) == 0U) || ((raw & 0x0100U) != 0U);
}

static uint32_t decodeErpmTelemetry(uint16_t raw)
{
    if (!isErpmTelemetryFrame(raw))
    {
        return 0;
    }

    uint16_t exponent = (raw >> 9U) & 0x7U;
    uint16_t mantissa = raw & 0x1FFU;
    uint32_t period = static_cast<uint32_t>(mantissa) << exponent;
    if (period == 0) return 0;
    // Period is in µs; convert to eRPM (electrical rotations per minute).
    return 60000000U / period;
}

// Constructor with GPIO number
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
    : _gpio(gpio),
      _mode(mode),
      _is_bidirectional(is_bidirectional),
      _motor_magnet_count(magnet_count),
      _dshot_timing(DSHOT_TIMING_US[_mode])
{
    // Pre-calculate timing and ratios for performance
    _preCalculateRMTTicks();
    _percent_to_throttle_ratio = (static_cast<float>(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)) / DSHOT_PERCENT_MAX;
}

// Constructor using pin number
DShotRMT::DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
    : DShotRMT(static_cast<gpio_num_t>(pin_nr), mode, is_bidirectional, magnet_count)
{
    // Delegates to primary constructor with type cast
}

// Destructor
DShotRMT::~DShotRMT()
{
    _cleanupRmtResources();
}

// Initialize DShotRMT
dshot_result_t DShotRMT::begin()
{
    dshot_result_t result = init_rmt_tx_channel(_gpio, &_rmt_tx_channel, _is_bidirectional);

    if (!result.success)
    {
        _cleanupRmtResources(); // Clean up any allocated resources on failure
        return result;
    }

    if (_is_bidirectional)
    {
        result = init_rmt_rx_channel(_gpio, &_rmt_rx_channel, &_rx_event_callbacks, this);
        if (!result.success)
        {
            _cleanupRmtResources(); // Clean up any allocated resources on failure
            return result;
        }
    }

    result = init_dshot_encoder(&_dshot_encoder, _rmt_ticks, _pulse_level, _idle_level);

    if (!result.success)
    {
        _cleanupRmtResources(); // Clean up any allocated resources on failure
        return result;
    }

    return {true, DSHOT_INIT_SUCCESS};
}

// Send throttle value
dshot_result_t DShotRMT::sendThrottle(uint16_t throttle)
{
    // Per DShot specification, a throttle value of 0 is a disarm command.
    if (throttle == 0)
    {
        _last_throttle = 0;
        return sendCommand(DSHOT_CMD_MOTOR_STOP);
    }

    // Constrain throttle to the valid DShot range.
    _last_throttle = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

    _packet = _buildDShotPacket(_last_throttle);
    return _sendPacket(_packet);
}

// Send throttle value as a percentage
dshot_result_t DShotRMT::sendThrottlePercent(float percent)
{
    if (percent < DSHOT_PERCENT_MIN || percent > DSHOT_PERCENT_MAX)
    {
        return {false, DSHOT_PERCENT_NOT_IN_RANGE};
    }

    // Map percent to DShot throttle range using pre-calculated ratio.
    uint16_t throttle = static_cast<uint16_t>(DSHOT_THROTTLE_MIN + _percent_to_throttle_ratio * percent);
    return sendThrottle(throttle);
}

// Sends a DShot command (0-47) to the ESC by accepting an integer value.
dshot_result_t DShotRMT::sendCommand(uint16_t command_value)
{
    // Validate the integer command value before casting.
    if (command_value < DSHOT_CMD_MOTOR_STOP || command_value > DSHOT_CMD_MAX_VALUE)
    {
        return {false, DSHOT_COMMAND_NOT_VALID};
    }
    return sendCommand(static_cast<dshotCommands_e>(command_value));
}

// Sends a DShot command (0-47) to the ESC.
dshot_result_t DShotRMT::sendCommand(dshotCommands_e command)
{
    uint16_t repeat_count = DEFAULT_CMD_REPEAT_COUNT;
    uint16_t delay_us = DEFAULT_CMD_DELAY_US;

    // Certain commands require more repetitions to be reliably accepted by the ESC.
    switch (command)
    {
    case DSHOT_CMD_MOTOR_STOP:
    case DSHOT_CMD_SAVE_SETTINGS:
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
        repeat_count = SETTINGS_COMMAND_REPEATS;
        delay_us = SETTINGS_COMMAND_DELAY_US;
        break;
    default:
        // For other commands, use default repeat and delay.
        break;
    }

    return sendCommand(command, repeat_count, delay_us);
}

// Sends a DShot command (0-47) to the ESC with a specified repeat count and delay.
dshot_result_t DShotRMT::sendCommand(dshotCommands_e command, uint16_t repeat_count, uint16_t delay_us)
{
    if (!_isValidCommand(command))
    {
        return {false, DSHOT_INVALID_COMMAND};
    }
    return _sendRepeatedCommand(static_cast<uint16_t>(command), repeat_count, delay_us);
}

// Get telemetry data
dshot_result_t DShotRMT::getTelemetry()
{
    dshot_result_t result = {false, DSHOT_TELEMETRY_FAILED};

    if (!_is_bidirectional)
    {
        result.result_code = DSHOT_BIDIR_NOT_ENABLED;
        return result;
    }

    if (_telemetry_ready_flag_atomic)
    {
        _telemetry_ready_flag_atomic = false; // Reset the flag
        uint16_t raw_erpm = _last_erpm_atomic;    // Read the atomic variable

        if (raw_erpm != DSHOT_NULL_PACKET && _motor_magnet_count >= MAGNETS_PER_POLE_PAIR)
        {
            // Decode the packed telemetry field into electrical RPM and derive
            // mechanical RPM from the configured magnet count.
            uint8_t pole_pairs = _motor_magnet_count / MAGNETS_PER_POLE_PAIR;
            result.telemetry_available = true;
            result.telemetry_data.rpm = raw_erpm;
            result.erpm = decodeErpmTelemetry(raw_erpm);
            result.motor_rpm = (result.erpm / pole_pairs);
            result.success = true;
            result.result_code = DSHOT_TELEMETRY_SUCCESS;
            return result;
        }
    }

    // Fall back to the full 10-byte telemetry frame only when no bidirectional
    // RPM sample is pending. This prevents stale full frames from masking the
    // live eRPM stream used during bringup.
    if (_full_telemetry_ready_flag_atomic)
    {
        _full_telemetry_ready_flag_atomic = false; // Reset the flag
        result.telemetry_data = _last_telemetry_data_atomic; // Read the atomic variable
        result.telemetry_available = true;

        result.erpm = decodeErpmTelemetry(result.telemetry_data.rpm);
        if (_motor_magnet_count >= MAGNETS_PER_POLE_PAIR) {
            uint8_t pole_pairs = _motor_magnet_count / MAGNETS_PER_POLE_PAIR;
            result.motor_rpm = result.erpm / pole_pairs;
        }
        
        result.success = true;
        result.result_code = DSHOT_TELEMETRY_DATA_AVAILABLE;
        return result;
    }

    return result;
}

// Reverse motor direction directly
dshot_result_t DShotRMT::setMotorSpinDirection(bool reversed)
{
    dshotCommands_e command = reversed ? dshotCommands_e::DSHOT_CMD_SPIN_DIRECTION_REVERSED : dshotCommands_e::DSHOT_CMD_SPIN_DIRECTION_NORMAL;
    return sendCommand(command, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_US);
}

dshot_result_t DShotRMT::sendCustomCommand(uint16_t command_value, uint16_t repeat_count, uint16_t delay_us)
{
    // Validate the integer command value.
    if (command_value < DSHOT_CMD_MIN || command_value > DSHOT_CMD_MAX)
    {
        return {false, DSHOT_COMMAND_NOT_VALID};
    }
    return _sendRepeatedCommand(command_value, repeat_count, delay_us);
}

// Writes settings to the ESC's non-volatile memory; use with caution.
dshot_result_t DShotRMT::saveESCSettings()
{
    return sendCommand(dshotCommands_e::DSHOT_CMD_SAVE_SETTINGS, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_US);
}

// Private helper to send a command value multiple times.
dshot_result_t DShotRMT::_sendRepeatedCommand(uint16_t value, uint16_t repeat_count, uint16_t delay_us)
{
    bool all_successful = true;
    dshot_result_t last_result = {true, DSHOT_COMMAND_SUCCESS};

    for (uint16_t i = 0; i < repeat_count; i++)
    {
        last_result = _sendRawDshotFrame(value);

        if (!last_result.success)
        {
            all_successful = false;
            break;
        }

        if (i < repeat_count - 1)
        {
            delayMicroseconds(delay_us);
        }
    }

    if (all_successful)
    {
        return {true, DSHOT_COMMAND_SUCCESS};
    }
    else
    {
        // Return the result from the failed transmission.
        return last_result;
    }
}

// Simple check for valid command range.
bool DShotRMT::_isValidCommand(dshotCommands_e command) const
{
    return (command >= dshotCommands_e::DSHOT_CMD_MOTOR_STOP && command <= DSHOT_CMD_MAX);
}

dshot_result_t DShotRMT::_sendRawDshotFrame(uint16_t value)
{
    _packet = _buildDShotPacket(value);
    return _sendPacket(_packet);
}

// Private Packet Management Functions
dshot_packet_t DShotRMT::_buildDShotPacket(const uint16_t &value) const
{
    dshot_packet_t packet = {};

    packet.throttle_value = value & DSHOT_THROTTLE_MAX;
    // value=0 must always have telemetric_request=0 — it is the zero-throttle
    // arming state, not a command. value=0 + telemetry=1 encodes as DShot
    // command #0 (MOTOR_STOP), which most ESCs do not count as the arming
    // handshake and will not arm from.
    packet.telemetric_request = (_is_bidirectional && (value & DSHOT_THROTTLE_MAX) != 0) ? 1 : 0;

    // The data for CRC calculation includes the 11-bit value and the 1-bit telemetry flag.
    uint16_t data_for_crc = (packet.throttle_value << 1) | packet.telemetric_request;
    packet.checksum = _calculateCRC(data_for_crc);

    return packet;
}

uint16_t DShotRMT::_buildDShotFrameValue(const dshot_packet_t &packet) const
{
    // Combine throttle, telemetry bit, and CRC into a single 16-bit frame.
    uint16_t data_and_telemetry = (packet.throttle_value << 1) | packet.telemetric_request;
    return (data_and_telemetry << 4) | packet.checksum;
}

uint16_t DShotRMT::_calculateCRC(const uint16_t &data) const
{
    // Standard DShot CRC calculation using XOR.
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & DSHOT_CRC_MASK;

    // For bidirectional DShot, the CRC is inverted per specification.
    if (_is_bidirectional)
    {
        crc = (~crc) & DSHOT_CRC_MASK;
    }
    return crc;
}

uint8_t DShotRMT::_calculateTelemetryCRC(const uint8_t *data, size_t len) const
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07; // DSHOT telemetry uses CRC-8 with polynomial 0x07.
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void DShotRMT::_extractTelemetryData(const uint8_t *raw_telemetry_bytes, dshot_telemetry_data_t &telemetry_data) const
{
    // Ensure the telemetry_data struct is cleared before filling.
    memset(&telemetry_data, 0, sizeof(dshot_telemetry_data_t));

    // Telemetry data is typically ordered as:
    // Byte 0: Temperature (signed 8-bit)
    // Byte 1-2: Voltage (16-bit, MSB first)
    // Byte 3-4: Current (16-bit, MSB first)
    // Byte 5-6: Consumption (16-bit, MSB first)
    // Byte 7-8: RPM (16-bit, MSB first)
    // Byte 9: CRC (8-bit) - checked separately

    telemetry_data.temperature = static_cast<int8_t>(raw_telemetry_bytes[0]);
    telemetry_data.voltage = (static_cast<uint16_t>(raw_telemetry_bytes[1]) << 8) | raw_telemetry_bytes[2];
    telemetry_data.current = (static_cast<uint16_t>(raw_telemetry_bytes[3]) << 8) | raw_telemetry_bytes[4];
    telemetry_data.consumption = (static_cast<uint16_t>(raw_telemetry_bytes[5]) << 8) | raw_telemetry_bytes[6];
    telemetry_data.rpm = (static_cast<uint16_t>(raw_telemetry_bytes[7]) << 8) | raw_telemetry_bytes[8];
}

void DShotRMT::_preCalculateRMTTicks()
{
    // Pre-calculate all timing values in RMT ticks to save CPU cycles during operation.
    _rmt_ticks.bit_length_ticks = static_cast<uint16_t>(_dshot_timing.bit_length_us * RMT_TICKS_PER_US);
    _rmt_ticks.t1h_ticks = static_cast<uint16_t>(_dshot_timing.t1h_lenght_us * RMT_TICKS_PER_US);
    _rmt_ticks.t0h_ticks = _rmt_ticks.t1h_ticks >> 1; // High time for a '0' bit is half of a '1' bit.
    _rmt_ticks.t1l_ticks = _rmt_ticks.bit_length_ticks - _rmt_ticks.t1h_ticks;
    _rmt_ticks.t0l_ticks = _rmt_ticks.bit_length_ticks - _rmt_ticks.t0h_ticks;

    // Calculate the minimum time required between frames to prevent signal collision.
    _frame_timer_us = (static_cast<uint64_t>(_dshot_timing.bit_length_us * DSHOT_BITS_PER_FRAME) << 1) + DSHOT_PADDING_US;

    if (_is_bidirectional)
    {
        _frame_timer_us = (_frame_timer_us << 1);
    }
}

// AM32 BDSHOT eRPM telemetry arrives as a 21-bit run-length encoded stream.
// Each RMT symbol contains two consecutive runs. Reconstruct the stream from
// those run lengths, then map the 5-bit GCR groups back to the 16-bit packet
// using the same decode table Betaflight/ArduPilot use.
uint16_t IRAM_ATTR DShotRMT::_decodeDShotFrame(const rmt_symbol_word_t *symbols, size_t num_symbols) const
{
    // Bidirectional DShot telemetry is transmitted at 5/4 the command bit rate,
    // so each GCR bit on the wire lasts only 4/5 of a DShot command bit period.
    // Quantizing by the command bit period (26 ticks for DShot300) pushes the
    // 2-vs-3-bit decision boundary out to 2.5*26 = 65 ticks, so real 3-bit runs
    // (~62 ticks, measured at bringup level 2) decode as 2 bits and corrupt the
    // frame — the cause of the ~90% CRC-fail rate. The measured GCR bit period
    // is ~20.5 ticks for DShot300; 4/5 of the command period lands dead-on.
    // Integer math only: this runs in the RMT RX ISR (no FPU saves there).
    const uint32_t cmd_ticks = _rmt_ticks.bit_length_ticks ? _rmt_ticks.bit_length_ticks : 26;
    const uint32_t unit = (cmd_ticks * 4u) / 5u;
    const uint32_t half = unit / 2;

    auto decode_value = [](uint32_t candidate) -> uint16_t {
        static const uint32_t decode[32] = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
            0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0
        };

        uint16_t decoded_value = decode[candidate & 0x1fU];
        decoded_value |= decode[(candidate >> 5U) & 0x1fU] << 4U;
        decoded_value |= decode[(candidate >> 10U) & 0x1fU] << 8U;
        decoded_value |= decode[(candidate >> 15U) & 0x1fU] << 12U;

        uint32_t csum = decoded_value;
        csum ^= (csum >> 8U);
        csum ^= (csum >> 4U);
        if ((csum & 0x0fU) != 0x0fU)
        {
            return DSHOT_NULL_PACKET;
        }

        // Strip the 4-bit CRC nibble. The caller wants the packed telemetry
        // payload so it can normalize the exponent/mantissa field.
        return decoded_value >> DSHOT_CRC_BIT_SHIFT;
    };

    auto build_value = [&](bool consume_final_tail) -> uint32_t {
        uint32_t value = 0;
        uint32_t bits = 0;

        for (size_t i = 0; i < num_symbols && bits < DSHOT_ERPM_FRAME_GCR_BITS; i++)
        {
            uint32_t runs[2] = {
                static_cast<uint32_t>((symbols[i].duration0 + half) / unit),
                static_cast<uint32_t>((symbols[i].duration1 + half) / unit),
            };

            for (size_t r = 0; r < 2 && bits < DSHOT_ERPM_FRAME_GCR_BITS; r++)
            {
                uint32_t run = runs[r];

                if (run == 0)
                {
                    // Some captures end with a zero-length second run. On the
                    // last symbol that can mean "the rest of the frame" rather
                    // than a literal zero-length interval.
                    if (consume_final_tail &&
                        i == num_symbols - 1 &&
                        r == 1 &&
                        bits < DSHOT_ERPM_FRAME_GCR_BITS)
                    {
                        run = DSHOT_ERPM_FRAME_GCR_BITS - bits;
                    }
                    else
                    {
                        continue;
                    }
                }

                uint32_t remaining = DSHOT_ERPM_FRAME_GCR_BITS - bits;
                if (run > remaining) run = remaining;

                value <<= run;
                value |= (1u << (run - 1u));
                bits += run;
            }
        }

        return bits == 0 ? DSHOT_NULL_PACKET : value;
    };

    const uint32_t values[] = {
        build_value(false),
        build_value(true),
    };

    for (uint32_t value : values)
    {
        if (value == DSHOT_NULL_PACKET)
        {
            continue;
        }

        const uint32_t candidates[] = {
            value << 1U,
            value,
            value >> 1U,
        };

        for (uint32_t candidate : candidates)
        {
            uint16_t raw_frame = decode_value(candidate);
            if (raw_frame == DSHOT_NULL_PACKET)
            {
                continue;
            }

            // A CRC-valid 0xFFF is the bidirectional "motor stopped / zero eRPM"
            // sentinel — a real frame, not an error. Accept it so an idle motor
            // reads as eRPM 0 (getTelemetry maps it to 0) instead of being tallied
            // as a CRC failure. An un-throttled ESC answers EVERY frame with 0xFFF,
            // which is why a single spinning motor previously showed a ~50% "fail"
            // rate — the other channel's valid stopped-frames were counted as fails.
            if (raw_frame != 0x0FFFU && !isErpmTelemetryFrame(raw_frame))
            {
                continue;
            }

            return raw_frame;
        }
    }

    return DSHOT_NULL_PACKET;
}

// Private Frame Processing Functions
dshot_result_t DShotRMT::_sendPacket(const dshot_packet_t &packet)
{
    // Ensure enough time has passed since the last transmission.
    if (!_isFrameIntervalElapsed())
    {
        return {true, DSHOT_NONE};
    }

    _encoded_frame_value = _buildDShotFrameValue(packet);

    // Byte-swap the 16-bit value for correct transmission order.
    // The RMT bytes encoder sends MSB of each byte first.
    uint16_t swapped_value = __builtin_bswap16(_encoded_frame_value);

    // The DShot frame is 16 bits, which is 2 bytes.
    size_t tx_size_bytes = sizeof(swapped_value);

    rmt_transmit_config_t tx_config = {.loop_count = 0}; // No automatic loops.

    // Disable RX before TX to prevent loopback — the inverted TX signal would
    // otherwise be captured by the RX channel on the same pin.
    if (_is_bidirectional)
    {
        if (rmt_disable(_rmt_rx_channel) != DSHOT_OK)
        {
            return {false, DSHOT_RECEIVER_FAILED};
        }
    }

    if (rmt_transmit(_rmt_tx_channel, _dshot_encoder, &swapped_value, tx_size_bytes, &tx_config) != DSHOT_OK)
    {
        return {false, DSHOT_TRANSMISSION_FAILED};
    }

    if (_is_bidirectional)
    {
        // Wait for TX to fully complete before enabling RX. rmt_transmit() is
        // async — if we arm the RX channel while TX is still sending, the RX
        // captures the tail of our own TX frame as loopback, giving the callback
        // a junk symbol count that neither matches 21 (eRPM) nor 110 (full telem),
        // so it silently discards every ESC response. DSHOT300 TX takes ~53 µs;
        // a 2 ms timeout is ample.
        rmt_tx_wait_all_done(_rmt_tx_channel, 2);

        if (rmt_enable(_rmt_rx_channel) != DSHOT_OK)
        {
            return {false, DSHOT_RECEIVER_FAILED};
        }

        // signal_range_min_ns: DSHOT300 "0-bit" LOW and "1-bit" HIGH periods are
        // both ~833 ns (25% of 3.33 µs). The original 800 ns limit was close
        // enough to filter these out, merging consecutive bit periods into fat
        // symbols and giving 6 symbols instead of 21. 200 ns keeps noise out
        // while passing every valid GCR pulse.
        // signal_range_max_ns: any idle > this ends the frame. 30 µs covers the
        // worst-case GCR run (multiple consecutive 0-bits at DSHOT300 speed).
        // signal_range_max_ns raised to 100µs so that the inter-frame gap
        // between TX end and the ESC starting its response (~25-30µs for
        // DShot300) does not prematurely terminate the RX capture.
        // The longest valid GCR run is 3 bits = ~10µs, well below 100µs.
        static const rmt_receive_config_t rmt_rx_config = {
            .signal_range_min_ns = 200,
            .signal_range_max_ns = 100000,
        };

        if (rmt_receive(_rmt_rx_channel, _rx_symbols, sizeof(_rx_symbols), &rmt_rx_config) != DSHOT_OK)
        {
            return {false, DSHOT_RECEIVER_FAILED};
        }
    }

    _recordFrameTransmissionTime(); // Reset the timer for the next frame.

    return {true, DSHOT_TRANSMISSION_SUCCESS};
}

// This function is placed in IRAM for high performance, as it may be
// called from an ISR context depending on RMT driver implementation.
// Timing Control Functions
bool IRAM_ATTR DShotRMT::_isFrameIntervalElapsed() const
{
    // Check if the minimum interval between frames has passed.
    uint64_t current_time = esp_timer_get_time();
    uint64_t elapsed = current_time - _last_transmission_time_us;
    return elapsed >= _frame_timer_us;
}

void DShotRMT::_recordFrameTransmissionTime()
{
    // Record the time of the current transmission.
    _last_transmission_time_us = esp_timer_get_time();
}

// Static Callback Functions
// Processes a full telemetry frame from the RMT RX ISR.
void IRAM_ATTR DShotRMT::_processFullTelemetryFrame(const rmt_symbol_word_t *symbols, size_t num_symbols)
{
    if (num_symbols != DSHOT_TELEMETRY_FULL_GCR_BITS)
    {
        return; // Incorrect number of symbols for a full telemetry frame.
    }

    uint8_t gcr_decoded_bytes[DSHOT_TELEMETRY_FRAME_LENGTH_BYTES + 1]; // 10 data bytes + 1 CRC byte.
    memset(gcr_decoded_bytes, 0, sizeof(gcr_decoded_bytes));

    uint8_t data_bit_idx = 0;
    for (size_t i = 0; i < DSHOT_TELEMETRY_FULL_GCR_BITS; i += 5)
    {
        uint8_t gcr_group_5bits = 0;
        for (size_t j = 0; j < 5; ++j)
        {
            if (i + j < DSHOT_TELEMETRY_FULL_GCR_BITS)
            {
                gcr_group_5bits = (gcr_group_5bits << 1) | ((symbols[i + j].duration0 > symbols[i + j].duration1) ? 1 : 0);
            }
        }

        uint8_t decoded_nibble; // 4 data bits.
        switch (gcr_group_5bits)
        {
        case 0b11110: decoded_nibble = 0b0000; break;
        case 0b01001: decoded_nibble = 0b0001; break;
        case 0b10100: decoded_nibble = 0b0010; break;
        case 0b10101: decoded_nibble = 0b0011; break;
        case 0b01010: decoded_nibble = 0b0100; break;
        case 0b01011: decoded_nibble = 0b0101; break;
        case 0b01110: decoded_nibble = 0b0110; break;
        case 0b01111: decoded_nibble = 0b0111; break;
        case 0b10010: decoded_nibble = 0b1000; break;
        case 0b10011: decoded_nibble = 0b1001; break;
        case 0b10110: decoded_nibble = 0b1010; break;
        case 0b10111: decoded_nibble = 0b1011; break;
        case 0b11010: decoded_nibble = 0b1100; break;
        case 0b11011: decoded_nibble = 0b1101; break;
        case 0b11100: decoded_nibble = 0b1110; break;
        case 0b11101: decoded_nibble = 0b1111; break;
        default: return; // Invalid GCR group, discard frame.
        }

        // Place the 4 decoded bits into the data_bytes array.
        for (int k = 3; k >= 0; --k)
        {
            if (data_bit_idx < (DSHOT_TELEMETRY_FRAME_LENGTH_BITS + DSHOT_TELEMETRY_CRC_LENGTH_BITS))
            {
                size_t byte_idx = data_bit_idx / 8;
                size_t bit_pos = data_bit_idx % 8;
                if (byte_idx < sizeof(gcr_decoded_bytes))
                {
                    gcr_decoded_bytes[byte_idx] |= ((decoded_nibble >> k) & 1) << (7 - bit_pos);
                }
                data_bit_idx++;
            }
        }
    }

    // The gcr_decoded_bytes array now contains the 10 telemetry bytes + 1 CRC byte.
    // Perform CRC validation.
    uint8_t received_crc = gcr_decoded_bytes[DSHOT_TELEMETRY_FRAME_LENGTH_BYTES];
    uint8_t calculated_crc = _calculateTelemetryCRC(gcr_decoded_bytes, DSHOT_TELEMETRY_FRAME_LENGTH_BYTES);

    if (received_crc == calculated_crc)
    {
        dshot_telemetry_data_t telemetry_data;
        // Extract from the first 10 bytes (excluding the CRC byte).
        _extractTelemetryData(gcr_decoded_bytes, telemetry_data);

        _last_telemetry_data_atomic.store(telemetry_data);
        _full_telemetry_ready_flag_atomic.store(true);
    }
}

// Debug counters — readable from dshot_print_telem_debug via extern
volatile uint32_t g_dshot_rx_cb_count    = 0; // times _on_rx_done fired at all
volatile uint32_t g_dshot_rx_sym_last    = 0; // num_symbols from last callback
volatile uint32_t g_dshot_rx_crc_ok      = 0; // frames that passed CRC
volatile uint32_t g_dshot_rx_crc_fail    = 0; // frames that failed CRC (decoded as 0)

// This function is called by the RMT driver's ISR when a frame is received.
bool IRAM_ATTR DShotRMT::_on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    DShotRMT *instance = static_cast<DShotRMT *>(user_data);

    if (edata)
    {
        g_dshot_rx_cb_count++;
        g_dshot_rx_sym_last = edata->num_symbols;

        // Snapshot for debug dump — copy before any processing.
        uint8_t n = edata->num_symbols < 48 ? edata->num_symbols : 48;
        memcpy(instance->_last_rx_snapshot, edata->received_symbols,
               n * sizeof(rmt_symbol_word_t));
        instance->_last_rx_num_symbols.store(n);

        if (edata->num_symbols == DSHOT_TELEMETRY_FULL_GCR_BITS)
        {
            instance->_processFullTelemetryFrame(edata->received_symbols, edata->num_symbols);
        }
        else if (edata->num_symbols >= 4 && edata->num_symbols <= 25)
        {
            // AM32 NRZ-M run-length encoding compresses the 21 GCR bits into
            // 4-10 RMT symbols rather than the 21 the original code expected.
            uint16_t erpm = instance->_decodeDShotFrame(edata->received_symbols, edata->num_symbols);

            if (erpm != DSHOT_NULL_PACKET)
            {
                g_dshot_rx_crc_ok++;
                instance->_last_erpm_atomic.store(erpm);
                instance->_telemetry_ready_flag_atomic.store(true);
            }
            else
            {
                g_dshot_rx_crc_fail++;
            }
        }
    }

    return false;
}

void DShotRMT::dumpLastRxFrame() const
{
    uint8_t n = _last_rx_num_symbols.load();
    uint32_t unit = _rmt_ticks.bit_length_ticks ? _rmt_ticks.bit_length_ticks : 26;
    Serial.printf("  RX frame: %u symbols  unit=%lu ticks/bit\n", n, unit);
    for (uint8_t i = 0; i < n; i++) {
        uint32_t d0 = _last_rx_snapshot[i].duration0;
        uint32_t d1 = _last_rx_snapshot[i].duration1;
        Serial.printf("  [%2u] d0=%4lu(l%u=%.1fb)  d1=%4lu(l%u=%.1fb)\n", i,
            d0, _last_rx_snapshot[i].level0, (float)d0/unit,
            d1, _last_rx_snapshot[i].level1, (float)d1/unit);
    }
}

void DShotRMT::_cleanupRmtResources()
{
    if (_rmt_tx_channel)
    {
        rmt_disable(_rmt_tx_channel);
        rmt_del_channel(_rmt_tx_channel);
        _rmt_tx_channel = nullptr;
    }

    if (_rmt_rx_channel)
    {
        rmt_disable(_rmt_rx_channel);
        rmt_del_channel(_rmt_rx_channel);
        _rmt_rx_channel = nullptr;
    }

    if (_dshot_encoder)
    {
        rmt_del_encoder(_dshot_encoder);
        _dshot_encoder = nullptr;
    }
}
