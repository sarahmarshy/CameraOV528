/* Copyright (c) 2016 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Originally Based on the (Grove_Serial_Camera_Kit) - https://github.com/Seeed-Studio/Grove_Serial_Camera_Kit
 */

#include<stdint.h>
#include<stdlib.h>
#include<string.h>

#include "CameraOV528.h"

#define camera_printf(...)

#define PIC_CLOCK_HZ                14745600
#define BOOT_BAUD                   9600

#define COMMAND_LENGTH              6
#define PIC_PAYLOAD_DATA_BEGIN      4
#define PIC_PAYLOAD_OVERHEAD        6
#define READ_WAKE_POS_INVALID       0xFFFFFFFF

#define DEFAULT_BAUD                115200

enum CameraCommand {
    INITIAL = 0x01,
    GET_PICTURE = 0x04,
    SNAPSHOT = 0x05,
    SET_PACKAGE_SIZE = 0x06,
    SET_BAUD_RATE = 0x07,
    RESET = 0x08,
    POWER_DOWN = 0x09,
    DATA = 0x0A,
    SYNC = 0x0D,
    ACK = 0x0E,
    NAK = 0x0F
};

enum GetSetting {
    GET_SNAPSHOT = 1,
    GET_PREVIEW_PICTURE = 2,
    GET_JPEG_PREVIEW_PICTURE = 3,
};

typedef struct {
    uint8_t header;
    uint8_t command;
    uint8_t param[4];
} camera_command_t;

CameraOV528::Resolution supported_resolutions[] = {
//    CameraOV528::RES_80x60,             //Does not work
    CameraOV528::RES_160x120,
    CameraOV528::RES_320x240,
    CameraOV528::RES_640x480,
};

CameraOV528::Format supported_formats[] = {
//    CameraOV528::FMT_2_BIT_GRAY_SCALE,  //Does not work
//    CameraOV528::FMT_4_BIT_GRAY_SCALE,
//    CameraOV528::FMT_8_BIT_GRAY_SCALE,
//    CameraOV528::FMT_2_BIT_COLOR,
//    CameraOV528::FMT_16_BIT,
    CameraOV528::FMT_JPEG,
};

static uint32_t divide_round_up(uint32_t dividen, uint32_t divisor);
static uint32_t min(uint32_t value1, uint32_t value2);
static uint32_t queue_used(uint32_t head, uint32_t tail, uint32_t size);

CameraOV528::CameraOV528(PinName rx, PinName tx) : _serial(rx, tx), _read_sem(0)
{
    _init_done = false;
    _resolution = RES_640x480;
    _format = FMT_JPEG;
    _baud = DEFAULT_BAUD;

    _dev_resolution = _resolution;
    _dev_format = _format;
    _dev_baud = _baud;

    picture_length = 0;
    picture_data_id = 0;
    picture_data_id_count = 0;
    memset(picture_buffer, 0, sizeof(picture_buffer));
    picture_buffer_size_limit = sizeof(picture_buffer);
    picture_buffer_pos = 0;
    picture_buffer_size = 0;

    memset(_read_buf, 0, sizeof(_read_buf));
    _read_buf_head = 0;
    _read_buf_tail = 0;
    _read_wake_pos = READ_WAKE_POS_INVALID;
}

CameraOV528::~CameraOV528()
{
    powerdown();
}

void CameraOV528::powerup()
{
    if (_init_done) {
        return;
    }

    _serial.attach(this, &CameraOV528::_rx_irq, SerialBase::RxIrq);

    // Attempt to connect at the desired baud
     _serial.baud(_baud);
     bool success = _init_sequence();

     // If unsuccessful try with boot baud
    if (!success) {
        _serial.baud(BOOT_BAUD);
        success = _init_sequence();
    }

    if (!success) {
        error("Unable to communicate with camera");
    }

    // Acknowledge the SYNC read in _init_sequence with an ACK
    _send_ack(SYNC, 0, 0, 0);
    camera_printf("\nCamera initialization done.\r\n");

    _set_baud(_baud);

    _set_fmt_and_res(_format, _resolution);
    _set_package_size(picture_buffer_size_limit);

    _init_done = false;
}

void CameraOV528::powerup(uint32_t baud)
{
    _baud = baud;
    powerup();
}

void CameraOV528::powerdown()
{
    if (!_init_done) {
        return;
    }

    if (!_send_cmd(POWER_DOWN, 0)) {
        error("Powerdown failed");
    }

    // Reset picture transfer variables
    picture_length = 0;
    picture_data_id = 0;
    picture_data_id_count = 0;
    memset(picture_buffer, 0, sizeof(picture_buffer));
    picture_buffer_size_limit = sizeof(picture_buffer);
    picture_buffer_pos = 0;
    picture_buffer_size = 0;

    _serial.attach(NULL, SerialBase::RxIrq);
    _flush_rx();

    _init_done = false;
}

void CameraOV528::take_picture(void)
{
    // Ensure driver is powered up
    powerup();

    // Update settings if any have changed
    if ((_dev_format != _format) || (_dev_resolution != _resolution)) {
        _set_fmt_and_res(_format, _resolution);
        _dev_format = _format;
        _dev_resolution = _resolution;
    }

    // Take snapshot
    camera_printf("Taking snapshot\r\n");
    if (!_send_cmd(SNAPSHOT, 0)) {
        error("Take snapshot failed");
    }

    // Start picture transfer
    camera_printf("Starting transfer\r\n");
    const GetSetting request = GET_JPEG_PREVIEW_PICTURE;
    if (!_send_cmd(GET_PICTURE, request)) {
        error("Get picture command failed");
    }
    camera_command_t resp = {0};
    uint32_t size_read = _read((uint8_t*)&resp, COMMAND_LENGTH, 1000);
    if (size_read != COMMAND_LENGTH) {
        error("Get picture response invalid");
    }
    if (resp.header != 0xAA)  error("Get picture response invalid sync");
    if (resp.command != DATA)  error("Get picture response invalid data");
    if (resp.param[0] != request)  error("Get picture response invalid content");
    picture_length = (resp.param[1] << 0) |
             (resp.param[2] << 8) |
             (resp.param[3] << 16);
    picture_data_id = 0;
    uint32_t payload_length = picture_buffer_size_limit - PIC_PAYLOAD_OVERHEAD;
    picture_data_id_count = divide_round_up(picture_length, payload_length);
}

uint32_t CameraOV528::get_picture_size()
{
    return picture_length;
}

uint32_t CameraOV528::read_picture_data(uint8_t * data, uint32_t size)
{
    uint32_t size_copied = 0;
    while (size_copied < size) {

        // Copy any data in the picture buffer
        uint32_t size_left = size - size_copied;
        uint32_t copy_size = min(picture_buffer_size, size_left);
        memcpy(data + size_copied, picture_buffer + picture_buffer_pos, copy_size);
        picture_buffer_pos += copy_size;
        picture_buffer_size -= copy_size;
        size_copied += copy_size;

        // If picture buffer is empty read more data
        if (0 == picture_buffer_size) {
            _read_picture_block();
        }

        // If there is still no more data to read then break from loop
        if (0 == picture_buffer_size) {
            break;
        }
    }

    return size_copied;
}

void CameraOV528::set_resolution(CameraOV528::Resolution resolution)
{
    const uint32_t count = sizeof(supported_resolutions) / sizeof(supported_resolutions[0]);
    bool valid_resolution = false;
    for (uint32_t i = 0; i < count; i++) {
        if (resolution == supported_resolutions[i]) {
            valid_resolution = true;
            break;
        }
    }

    if (!valid_resolution) {
        error("Invalid resolution");
    }

    _resolution = resolution;

}

void CameraOV528::set_format(CameraOV528::Format format)
{
    const uint32_t count = sizeof(supported_formats) / sizeof(supported_formats[0]);
    bool valid_format = false;
    for (uint32_t i = 0; i < count; i++) {
        if (format == supported_formats[i]) {
            valid_format = true;
            break;
        }
    }

    if (!valid_format) {
        error("Invalid format");
    }

    _format = format;
}

void CameraOV528::_set_baud(uint32_t baud)
{
    // Baud Rate = 14.7456MHz/(2*(2nd divider +1))/(2*(1st divider+1))
    uint32_t div2 = 1;
    uint32_t div1 = PIC_CLOCK_HZ / _baud / (2 * (div2 + 1)) / 2 - 1;
    // Assert no rounding errors
    MBED_ASSERT(PIC_CLOCK_HZ / ( 2 * (div2 + 1) ) / ( 2 * (div1 + 1)) == _baud);
    if (!_send_cmd(SET_BAUD_RATE, div1, div2)) {
        error("_set_baud failed");
    }
    _serial.baud(_baud);
}

void CameraOV528::_set_package_size(uint32_t size)
{
    camera_printf("_set_package_size(%lu)\r\n", size);
    uint8_t size_low = (size >> 0) & 0xff;
    uint8_t size_high = (size >> 8) & 0xff;
    if (!_send_cmd(SET_PACKAGE_SIZE, 0x08, size_low, size_high, 0)) {
        error("_set_package_size failed");
    }
}

void CameraOV528::_set_fmt_and_res(Format fmt, Resolution res)
{
    if (!_send_cmd(INITIAL, 0x00, _format, 0x00, _resolution)) {
        error("_set_fmt_and_res failed");
    }
}

void CameraOV528::_read_picture_block()
{
    const uint32_t payload_length = picture_buffer_size_limit - PIC_PAYLOAD_OVERHEAD;
    if (picture_data_id >= picture_data_id_count) {
        // Transfer complete
        return;
    }

    // Send an ACK to indicate that the next block id should be sent
    uint8_t id_low = (picture_data_id >> 0) & 0xff;
    uint8_t id_high = (picture_data_id >> 8) & 0xff;
    _send_ack(0x00, 0x00, id_low, id_high);

    // Read image data
    memset(picture_buffer, 0, sizeof(picture_buffer));
    uint32_t size_left = picture_length - payload_length * picture_data_id;
    uint32_t size_to_read = min(size_left, payload_length) + PIC_PAYLOAD_OVERHEAD;
    uint32_t size_read = _read(picture_buffer, size_to_read);
    if (size_read != size_to_read) {
        error("Image data protocol error");
    }

    // Validate checksum
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < size_read - 2; i++) {
        checksum += picture_buffer[i];
    }
    if (picture_buffer[size_read - 2] != checksum) {
        error("Image data checksum failure");
    }

    // Update buffer information
    picture_buffer_pos = PIC_PAYLOAD_DATA_BEGIN;
    picture_buffer_size = size_read - PIC_PAYLOAD_OVERHEAD;

    // If this is the last packet then send the completion ACK
    if (picture_data_id + 1 == picture_data_id_count) {
        _send_ack(0x00, 0x00, 0xF0, 0xF0);
    }

    // Increment position
    camera_printf("%lu of %lu\r\n", picture_data_id + 1, picture_data_id_count);
    camera_printf("size left %lu\r\n",
                  picture_length - payload_length * picture_data_id +
                  PIC_PAYLOAD_OVERHEAD - size_read);
    picture_data_id++;
}

void CameraOV528::_send_ack(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
{
    camera_command_t cmd = {0};
    cmd.header = 0xAA;
    cmd.command = ACK;
    cmd.param[0] = p1;
    cmd.param[1] = p2;
    cmd.param[2] = p3;
    cmd.param[3] = p4;
    _send((uint8_t*)&cmd, COMMAND_LENGTH);
}

void CameraOV528::_rx_irq(void)
{
    if(_serial.readable()) {

        // Check for overflow
        if (_read_buf_head + 1 == _read_buf_tail) {
            error("RX buffer overflow");
        }

        // Add data
        _read_buf[_read_buf_head] = (uint8_t)_serial.getc();
        _read_buf_head = (_read_buf_head + 1) % sizeof(_read_buf);

        // Check if thread should be woken
        if (_read_buf_head == _read_wake_pos) {
            _read_sem.release();
            _read_wake_pos = READ_WAKE_POS_INVALID;
        }
    }
}

bool CameraOV528::_send(const uint8_t *data, uint32_t size, uint32_t timeout_ms)
{
    for (uint32_t i = 0; i < size; i++) {
        _serial.putc(data[i]);
    }
    return true;
}

uint32_t CameraOV528::_read(uint8_t *data, uint32_t size, uint32_t timeout_ms)
{
    MBED_ASSERT(size < sizeof(_read_buf));
    MBED_ASSERT(0 == _read_sem.wait(0));

    core_util_critical_section_enter();

    // Atomically set wakeup condition
    uint32_t size_available = queue_used(_read_buf_head, _read_buf_tail, sizeof(_read_buf));
    if (size_available >= size) {
        _read_wake_pos = READ_WAKE_POS_INVALID;
    } else {
        _read_wake_pos = (_read_buf_tail + size) % sizeof(_read_buf);
    }

    core_util_critical_section_exit();

    // Wait until the requested number of bytes are available
    if (_read_wake_pos != READ_WAKE_POS_INVALID) {
        int32_t tokens = _read_sem.wait(timeout_ms);
        if (tokens <= 0) {
            // Timeout occurred so make sure semaphore is cleared
            _read_wake_pos = READ_WAKE_POS_INVALID;
            _read_sem.wait(0);
        } else {
            // If the semaphore was signaled then the requested number of
            // bytes were read
            MBED_ASSERT(queue_used(_read_buf_head, _read_buf_tail, sizeof(_read_buf)) >= size);
        }
    }

    // Copy bytes
    size_available = queue_used(_read_buf_head, _read_buf_tail, sizeof(_read_buf));
    uint32_t read_size = min(size_available, size);
    for (uint32_t i = 0; i < read_size; i++) {
        data[i] = _read_buf[_read_buf_tail];
        _read_buf_tail = (_read_buf_tail + 1) % sizeof(_read_buf);
    }

    return read_size;
}


void CameraOV528::_flush_rx(void)
{

    core_util_critical_section_enter();
    _read_buf_head = 0;
    _read_buf_tail = 0;
    _read_wake_pos = 0;
    core_util_critical_section_exit();
}

bool CameraOV528::_send_cmd(uint8_t cmd, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
{
    _flush_rx();

    camera_command_t command = {0};
    command.header = 0xAA;
    command.command = cmd;
    command.param[0] = p1;
    command.param[1] = p2;
    command.param[2] = p3;
    command.param[3] = p4;
    _send((uint8_t*)&command, COMMAND_LENGTH);

    camera_command_t resp = {0};
    uint32_t len = _read((uint8_t*)&resp, COMMAND_LENGTH);
    if (COMMAND_LENGTH != len) {
        camera_printf("Wrong command response length %lu\r\n", len);
    }
    if ((resp.header != 0xAA) || (resp.command != ACK) || (resp.param[0] != command.command)) {
        camera_printf("Invalid command: %i, %i, %i\r\n", resp.header, resp.command, resp.param[0]);
        return false;
    }
    return true;
}

bool CameraOV528::_init_sequence()
{
    camera_printf("connecting to camera...");
    bool success = false;
    for (uint32_t i = 0; i < 4; i++) {
        camera_printf(".");

        // Send SYNC command repeatedly
        if (!_send_cmd(SYNC, 0, 0, 0, 0)) {
            continue;
        }
        // Device should send back SYNC command
        camera_command_t resp = {0};
        if (_read((uint8_t*)&resp, COMMAND_LENGTH, 500) != COMMAND_LENGTH) {
            continue;
        }
        if (resp.header != 0xAA) continue;
        if (resp.command != SYNC) continue;
        if (resp.param[0] != 0) continue;
        if (resp.param[1] != 0) continue;
        if (resp.param[2] != 0) continue;
        if (resp.param[3] != 0) continue;
        success = true;
        break;
    }
    return success;
}

static uint32_t divide_round_up(uint32_t dividen, uint32_t divisor)
{
    return (dividen + divisor - 1) / divisor;
}

static uint32_t min(uint32_t value1, uint32_t value2)
{
    return value1 < value2 ? value1 : value2;
}

static uint32_t queue_used(uint32_t head, uint32_t tail, uint32_t size)
{
    if (head < tail) {
        return head + size - tail;
    } else {
        return head - tail;
    }
}
