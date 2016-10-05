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
#ifndef MBED_CAMERAOV528_H
#define MBED_CAMERAOV528_H

#include "mbed.h"
#include "rtos.h"

class Camera {
public:
    Camera(void) {};
    virtual ~Camera(void) {};
    virtual void powerup(void) = 0;
    virtual void powerdown(void) = 0;
    virtual void take_picture(void) = 0;
    virtual uint32_t get_picture_size(void) = 0;
    virtual uint32_t read_picture_data(uint8_t *data, uint32_t size) = 0;
};

class CameraOV528 : public Camera {

public:
    enum Resolution {
//        RES_80x60       = 0x01,   // Does not work
        RES_160x120     = 0x03,
        RES_320x240     = 0x05,
        RES_640x480     = 0x07,
    };

    enum Format {
//        FMT_2_BIT_GRAY_SCALE = 1, //Does not work
//        FMT_4_BIT_GRAY_SCALE = 2,
//        FMT_8_BIT_GRAY_SCALE = 3,
//        FMT_2_BIT_COLOR = 5,
//        FMT_16_BIT = 6,
        FMT_JPEG = 7,
    };

    CameraOV528(PinName rx, PinName tx);
    virtual ~CameraOV528(void);
    virtual void powerup(void);
    virtual void powerup(uint32_t baud);
    virtual void powerdown(void);
    virtual void take_picture(void);
    virtual uint32_t get_picture_size(void);
    virtual uint32_t read_picture_data(uint8_t *data, uint32_t size);

    void set_resolution(Resolution resolution);
    void set_format(Format format);
    void set_baud(uint32_t baud);

private:

    bool _init_sequence();
    void _set_baud(uint32_t baud);
    void _set_package_size(uint32_t size);
    void _set_fmt_and_res(Format fmt, Resolution res);
    void _read_picture_block();

    void _rx_irq(void);

    bool _send(const uint8_t *data, uint32_t size, uint32_t timeout_ms=500);
    uint32_t _read(uint8_t * data, uint32_t size, uint32_t timeout_ms=500);
    void _flush_rx(void);

    bool _send_cmd(uint8_t cmd, uint8_t p1=0, uint8_t p2=0, uint8_t p3=0, uint8_t p4=0);
    void _send_ack(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4);

    RawSerial _serial;

    bool _init_done;
    Resolution _resolution;
    Format _format;
    uint32_t _baud;

    Resolution _dev_resolution;
    Format _dev_format;
    uint32_t _dev_baud;

    uint32_t picture_length;
    uint32_t picture_data_id;
    uint32_t picture_data_id_count;
    uint8_t picture_buffer[128];
    uint32_t picture_buffer_size_limit;
    uint32_t picture_buffer_pos;
    uint32_t picture_buffer_size;

    uint8_t _read_buf[sizeof(picture_buffer) + 1];
    uint32_t _read_buf_head;
    uint32_t _read_buf_tail;
    uint32_t _read_wake_pos;
    Semaphore _read_sem;
};

#endif
