/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 */

#pragma once

#include "common/streambuf.h"
#include "msp/msp.h"

enum {
    MSP_DP_GHOST = 0x80,
    MSP_GHOST_DP_VERSION_1_0 = 0x10,
    MSP_GHOST_DP_HEADER_SIZE = 10,
};

typedef enum {
    MSP_GHOST_DP_ENDPOINT_UNSPECIFIED = 0,
    MSP_GHOST_DP_ENDPOINT_FLIGHT_CONTROLLER = 1,
    MSP_GHOST_DP_ENDPOINT_VRX = 2,
    MSP_GHOST_DP_ENDPOINT_CONFIGURATOR = 3,
    MSP_GHOST_DP_ENDPOINT_BROADCAST = 255,
} mspGhostDpEndpoint_e;

typedef enum {
    MSP_GHOST_DP_FLAG_REQUEST = 1 << 0,
    MSP_GHOST_DP_FLAG_RESPONSE = 1 << 1,
    MSP_GHOST_DP_FLAG_MORE = 1 << 2,
    MSP_GHOST_DP_FLAG_ERROR = 1 << 3,
    MSP_GHOST_DP_FLAG_VOLATILE = 1 << 4,
} mspGhostDpFlag_e;

typedef enum {
    MSP_GHOST_DP_HELLO_REQUEST = 0x01,
    MSP_GHOST_DP_HELLO_RESPONSE = 0x02,
} mspGhostDpMessageType_e;

typedef enum {
    MSP_GHOST_DP_STATUS_OK = 0,
    MSP_GHOST_DP_STATUS_BAD_LENGTH = 1,
    MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION = 2,
} mspGhostDpStatus_e;

mspResult_e mspGhostDpProcessCommand(sbuf_t *src, sbuf_t *dst);
