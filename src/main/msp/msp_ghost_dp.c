/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_MSP_DISPLAYPORT

#include "drivers/system.h"

#include "msp/msp_ghost_dp.h"
#include "msp/msp_serial.h"

typedef struct mspGhostDpHeader_s {
    uint8_t version;
    uint8_t messageType;
    uint8_t flags;
    uint8_t source;
    uint8_t destination;
    uint16_t sessionId;
    uint16_t exchangeId;
} mspGhostDpHeader_t;

static uint32_t ghostBootId;
static uint16_t ghostSessionId;

static void mspGhostDpInitSession(void)
{
    if (ghostSessionId != 0) {
        return;
    }

    /*
     * The MCU UID makes the value flight-controller-specific and the cycle
     * counter makes it boot-specific without requiring a flash write. The
     * identifiers only detect a restarted session; they are not security
     * tokens or cryptographic random numbers.
     */
    uint32_t value = getCycleCounter() ^ U_ID_0;
    value ^= (U_ID_1 << 7) | (U_ID_1 >> 25);
    value ^= (U_ID_2 << 13) | (U_ID_2 >> 19);
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    if (value == 0) {
        value = 1;
    }

    ghostBootId = value;
    ghostSessionId = (uint16_t)(value ^ (value >> 16));
    if (ghostSessionId == 0) {
        ghostSessionId = 1;
    }
}

static void mspGhostDpWriteHeader(sbuf_t *dst, uint8_t messageType,
    uint8_t flags, uint8_t destination, uint16_t sessionId,
    uint16_t exchangeId)
{
    sbufWriteU8(dst, MSP_DP_GHOST);
    sbufWriteU8(dst, MSP_GHOST_DP_VERSION_1_0);
    sbufWriteU8(dst, messageType);
    sbufWriteU8(dst, flags);
    sbufWriteU8(dst, MSP_GHOST_DP_ENDPOINT_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, destination);
    sbufWriteU16(dst, sessionId);
    sbufWriteU16(dst, exchangeId);
}

static void mspGhostDpWriteHelloResponse(sbuf_t *dst,
    const mspGhostDpHeader_t *request, mspGhostDpStatus_e status)
{
    mspGhostDpInitSession();

    uint8_t flags = MSP_GHOST_DP_FLAG_RESPONSE;
    if (status != MSP_GHOST_DP_STATUS_OK) {
        flags |= MSP_GHOST_DP_FLAG_ERROR;
    }
    mspGhostDpWriteHeader(dst, MSP_GHOST_DP_HELLO_RESPONSE, flags,
        request->source, ghostSessionId, request->exchangeId);

    sbufWriteU8(dst, status);
    sbufWriteU32(dst, ghostBootId);

    /* Stable 96-bit STM32 UID padded to the protocol's 16-byte field. */
    sbufWriteU32(dst, U_ID_0);
    sbufWriteU32(dst, U_ID_1);
    sbufWriteU32(dst, U_ID_2);
    sbufWriteU32(dst, 0);

    /* Phase 1 implements discovery only. */
    sbufWriteU32(dst, 0);                    // catalog_hash
    sbufWriteU32(dst, 0);                    // capability_flags
    sbufWriteU16(dst, MSP_PORT_INBUF_SIZE);  // max_payload
    sbufWriteU32(dst, 0);                    // max_stream_bps
    sbufWriteU8(dst, 0);                     // max_slots
    sbufWriteU8(dst, 0);                     // lease_seconds
}

mspResult_e mspGhostDpProcessCommand(sbuf_t *src, sbuf_t *dst)
{
    const unsigned payloadLength = sbufBytesRemaining(src);
    if (payloadLength == 0 || sbufReadU8(src) != MSP_DP_GHOST) {
        return MSP_RESULT_CMD_UNKNOWN;
    }

    if (payloadLength < MSP_GHOST_DP_HEADER_SIZE) {
        return MSP_RESULT_ERROR;
    }

    const mspGhostDpHeader_t request = {
        .version = sbufReadU8(src),
        .messageType = sbufReadU8(src),
        .flags = sbufReadU8(src),
        .source = sbufReadU8(src),
        .destination = sbufReadU8(src),
        .sessionId = sbufReadU16(src),
        .exchangeId = sbufReadU16(src),
    };

    if (request.messageType != MSP_GHOST_DP_HELLO_REQUEST ||
        !(request.flags & MSP_GHOST_DP_FLAG_REQUEST) ||
        (request.flags & MSP_GHOST_DP_FLAG_RESPONSE) ||
        (request.destination != MSP_GHOST_DP_ENDPOINT_FLIGHT_CONTROLLER &&
         request.destination != MSP_GHOST_DP_ENDPOINT_BROADCAST)) {
        return MSP_RESULT_ERROR;
    }

    mspGhostDpStatus_e status = MSP_GHOST_DP_STATUS_OK;
    if ((request.version >> 4) != (MSP_GHOST_DP_VERSION_1_0 >> 4)) {
        status = MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION;
    } else if (sbufBytesRemaining(src) != 0) {
        status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
    }

    mspGhostDpWriteHelloResponse(dst, &request, status);
    return MSP_RESULT_ACK;
}

#endif // USE_MSP_DISPLAYPORT
