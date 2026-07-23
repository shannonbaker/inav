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
#include <string.h>

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

typedef struct mspGhostDpFieldDescriptor_s {
    uint16_t id;
    uint8_t valueType;
    uint8_t unit;
    int8_t scaleExponent;
    uint16_t flags;
    uint16_t maximumRateHz;
    uint16_t nativeRateHz;
    uint8_t instanceCount;
    const char *name;
} mspGhostDpFieldDescriptor_t;

#define FIELD_SIGNED MSP_GHOST_DP_FIELD_SIGNED
#define FIELD_INVALID MSP_GHOST_DP_FIELD_TEMPORARILY_INVALID
#define RC_FIELD(channel, id) \
    { id, MSP_GHOST_DP_VALUE_U16, MSP_GHOST_DP_UNIT_SECOND, -6, \
      FIELD_INVALID, 100, 100, 1, "RC" #channel }

static const mspGhostDpFieldDescriptor_t fieldCatalog[] = {
    { 1, MSP_GHOST_DP_VALUE_I16, MSP_GHOST_DP_UNIT_DEGREE, -1,
      FIELD_SIGNED, 100, 100, 1, "PITCH" },
    { 2, MSP_GHOST_DP_VALUE_I16, MSP_GHOST_DP_UNIT_DEGREE, -1,
      FIELD_SIGNED, 100, 100, 1, "ROLL" },
    { 3, MSP_GHOST_DP_VALUE_U16, MSP_GHOST_DP_UNIT_DEGREE, -1,
      0, 100, 100, 1, "HEADING" },
#ifdef USE_GPS
    { 4, MSP_GHOST_DP_VALUE_I32, MSP_GHOST_DP_UNIT_DEGREE, -7,
      FIELD_SIGNED | FIELD_INVALID, 10, 10, 1, "LATITUDE" },
    { 5, MSP_GHOST_DP_VALUE_I32, MSP_GHOST_DP_UNIT_DEGREE, -7,
      FIELD_SIGNED | FIELD_INVALID, 10, 10, 1, "LONGITUDE" },
    { 6, MSP_GHOST_DP_VALUE_I32, MSP_GHOST_DP_UNIT_METRE, -2,
      FIELD_SIGNED | FIELD_INVALID, 10, 10, 1, "GPS_ALTITUDE" },
    { 7, MSP_GHOST_DP_VALUE_U16, MSP_GHOST_DP_UNIT_METRES_PER_SECOND, -2,
      FIELD_INVALID, 10, 10, 1, "GROUND_SPEED" },
#endif
    { 8, MSP_GHOST_DP_VALUE_U16, MSP_GHOST_DP_UNIT_VOLT, -3,
      FIELD_INVALID, 20, 10, 1, "BATTERY_VOLTAGE" },
    { 9, MSP_GHOST_DP_VALUE_I32, MSP_GHOST_DP_UNIT_AMPERE, -2,
      FIELD_SIGNED | FIELD_INVALID, 20, 10, 1, "BATTERY_CURRENT" },
    { 10, MSP_GHOST_DP_VALUE_I32, MSP_GHOST_DP_UNIT_AMPERE_HOUR, -3,
      FIELD_SIGNED | FIELD_INVALID, 20, 10, 1, "BATTERY_MAH" },
#ifdef USE_GPS
    { 11, MSP_GHOST_DP_VALUE_U8, MSP_GHOST_DP_UNIT_COUNT, 0,
      FIELD_INVALID, 10, 10, 1, "GPS_SATELLITES" },
    { 12, MSP_GHOST_DP_VALUE_U16, MSP_GHOST_DP_UNIT_DEGREE, -1,
      FIELD_INVALID, 10, 10, 1, "HOME_BEARING" },
#endif
    { 13, MSP_GHOST_DP_VALUE_BOOL, MSP_GHOST_DP_UNIT_NONE, 0,
      0, 10, 10, 1, "HEADING_VALID" },
#ifdef USE_GPS
    { 14, MSP_GHOST_DP_VALUE_BOOL, MSP_GHOST_DP_UNIT_NONE, 0,
      0, 10, 10, 1, "GPS_FIX" },
    { 15, MSP_GHOST_DP_VALUE_BOOL, MSP_GHOST_DP_UNIT_NONE, 0,
      0, 10, 10, 1, "HOME_VALID" },
#endif
    RC_FIELD(1, 32),
    RC_FIELD(2, 33),
    RC_FIELD(3, 34),
    RC_FIELD(4, 35),
    RC_FIELD(5, 36),
    RC_FIELD(6, 37),
    RC_FIELD(7, 38),
    RC_FIELD(8, 39),
    RC_FIELD(9, 40),
    RC_FIELD(10, 41),
    RC_FIELD(11, 42),
    RC_FIELD(12, 43),
    RC_FIELD(13, 44),
    RC_FIELD(14, 45),
    RC_FIELD(15, 46),
    RC_FIELD(16, 47),
    RC_FIELD(17, 48),
    RC_FIELD(18, 49),
};

#undef RC_FIELD
#undef FIELD_INVALID
#undef FIELD_SIGNED

static uint32_t ghostBootId;
static uint16_t ghostSessionId;
static uint32_t ghostCatalogHash;

static uint32_t fnv1aByte(uint32_t hash, uint8_t value)
{
    return (hash ^ value) * 16777619u;
}

static uint32_t mspGhostDpCatalogHash(void)
{
    if (ghostCatalogHash != 0) {
        return ghostCatalogHash;
    }

    uint32_t hash = 2166136261u;
    for (unsigned index = 0; index < sizeof(fieldCatalog) / sizeof(fieldCatalog[0]); ++index) {
        const mspGhostDpFieldDescriptor_t *field = &fieldCatalog[index];
        hash = fnv1aByte(hash, field->id);
        hash = fnv1aByte(hash, field->id >> 8);
        hash = fnv1aByte(hash, field->valueType);
        hash = fnv1aByte(hash, field->unit);
        hash = fnv1aByte(hash, (uint8_t)field->scaleExponent);
        hash = fnv1aByte(hash, field->flags);
        hash = fnv1aByte(hash, field->flags >> 8);
        hash = fnv1aByte(hash, field->maximumRateHz);
        hash = fnv1aByte(hash, field->maximumRateHz >> 8);
        hash = fnv1aByte(hash, field->nativeRateHz);
        hash = fnv1aByte(hash, field->nativeRateHz >> 8);
        hash = fnv1aByte(hash, field->instanceCount);
        const uint8_t nameLength = strlen(field->name);
        hash = fnv1aByte(hash, nameLength);
        for (unsigned nameIndex = 0; nameIndex < nameLength; ++nameIndex) {
            hash = fnv1aByte(hash, field->name[nameIndex]);
        }
    }

    /* Zero is reserved for an unavailable catalogue. */
    ghostCatalogHash = hash != 0 ? hash : 1;
    return ghostCatalogHash;
}

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

static uint8_t responseFlags(mspGhostDpStatus_e status)
{
    return MSP_GHOST_DP_FLAG_RESPONSE |
        (status == MSP_GHOST_DP_STATUS_OK ? 0 : MSP_GHOST_DP_FLAG_ERROR);
}

static void mspGhostDpWriteHelloResponse(sbuf_t *dst,
    const mspGhostDpHeader_t *request, mspGhostDpStatus_e status)
{
    mspGhostDpInitSession();
    mspGhostDpWriteHeader(dst, MSP_GHOST_DP_HELLO_RESPONSE,
        responseFlags(status), request->source, ghostSessionId,
        request->exchangeId);

    sbufWriteU8(dst, status);
    sbufWriteU32(dst, ghostBootId);

    /* Stable 96-bit STM32 UID padded to the protocol's 16-byte field. */
    sbufWriteU32(dst, U_ID_0);
    sbufWriteU32(dst, U_ID_1);
    sbufWriteU32(dst, U_ID_2);
    sbufWriteU32(dst, 0);

    sbufWriteU32(dst, mspGhostDpCatalogHash());
    sbufWriteU32(dst, 1u << 0);              // Field catalogue capability
    sbufWriteU16(dst, MSP_PORT_INBUF_SIZE);  // max_payload
    sbufWriteU32(dst, 0);                    // max_stream_bps
    sbufWriteU8(dst, 0);                     // max_slots
    sbufWriteU8(dst, 0);                     // lease_seconds
}

static void mspGhostDpWriteFieldRecord(sbuf_t *dst,
    const mspGhostDpFieldDescriptor_t *field)
{
    const uint8_t nameLength = strlen(field->name);
    sbufWriteU8(dst, 13 + nameLength); // Bytes following record_length.
    sbufWriteU16(dst, field->id);
    sbufWriteU8(dst, field->valueType);
    sbufWriteU8(dst, field->unit);
    sbufWriteU8(dst, (uint8_t)field->scaleExponent);
    sbufWriteU16(dst, field->flags);
    sbufWriteU16(dst, field->maximumRateHz);
    sbufWriteU16(dst, field->nativeRateHz);
    sbufWriteU8(dst, field->instanceCount);
    sbufWriteU8(dst, nameLength);
    sbufWriteData(dst, field->name, nameLength);
}

static void mspGhostDpWriteCatalogResponse(sbuf_t *dst,
    const mspGhostDpHeader_t *request, mspGhostDpStatus_e status,
    uint16_t startFieldId, uint8_t maximumRecords)
{
    mspGhostDpWriteHeader(dst, MSP_GHOST_DP_FIELD_CATALOG_RESPONSE,
        responseFlags(status), request->source, ghostSessionId,
        request->exchangeId);
    sbufWriteU8(dst, status);
    sbufWriteU32(dst, mspGhostDpCatalogHash());

    uint8_t *nextFieldIdPtr = sbufPtr(dst);
    sbufWriteU16(dst, 0);
    uint8_t *recordCountPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);

    if (status != MSP_GHOST_DP_STATUS_OK) {
        return;
    }

    unsigned index = 0;
    const unsigned catalogSize = sizeof(fieldCatalog) / sizeof(fieldCatalog[0]);
    while (index < catalogSize && fieldCatalog[index].id < startFieldId) {
        ++index;
    }

    unsigned payloadLength = MSP_GHOST_DP_HEADER_SIZE + 8;
    uint8_t recordCount = 0;
    while (index < catalogSize && recordCount < maximumRecords) {
        const mspGhostDpFieldDescriptor_t *field = &fieldCatalog[index];
        const unsigned wireLength = 1 + 13 + strlen(field->name);
        if (payloadLength + wireLength > MSP_GHOST_DP_NEGOTIATION_PAYLOAD_MAX) {
            break;
        }
        mspGhostDpWriteFieldRecord(dst, field);
        payloadLength += wireLength;
        ++recordCount;
        ++index;
    }

    *recordCountPtr = recordCount;
    if (index < catalogSize) {
        const uint16_t nextFieldId = fieldCatalog[index].id;
        nextFieldIdPtr[0] = nextFieldId;
        nextFieldIdPtr[1] = nextFieldId >> 8;
    }
}

static bool mspGhostDpHeaderIsRequest(const mspGhostDpHeader_t *request)
{
    return (request->flags & MSP_GHOST_DP_FLAG_REQUEST) &&
        !(request->flags & MSP_GHOST_DP_FLAG_RESPONSE) &&
        (request->destination == MSP_GHOST_DP_ENDPOINT_FLIGHT_CONTROLLER ||
         request->destination == MSP_GHOST_DP_ENDPOINT_BROADCAST);
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
    if (!mspGhostDpHeaderIsRequest(&request)) {
        return MSP_RESULT_ERROR;
    }

    const bool versionSupported =
        (request.version >> 4) == (MSP_GHOST_DP_VERSION_1_0 >> 4);
    switch (request.messageType) {
    case MSP_GHOST_DP_HELLO_REQUEST: {
        mspGhostDpStatus_e status = MSP_GHOST_DP_STATUS_OK;
        if (!versionSupported) {
            status = MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION;
        } else if (sbufBytesRemaining(src) != 0) {
            status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        }
        mspGhostDpWriteHelloResponse(dst, &request, status);
        return MSP_RESULT_ACK;
    }

    case MSP_GHOST_DP_FIELD_CATALOG_REQUEST: {
        mspGhostDpStatus_e status = MSP_GHOST_DP_STATUS_OK;
        uint16_t startFieldId = 0;
        uint8_t maximumRecords = 0;
        if (!versionSupported) {
            status = MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION;
        } else if (ghostSessionId == 0 || request.sessionId != ghostSessionId) {
            status = MSP_GHOST_DP_STATUS_INVALID_SESSION;
        } else if (sbufBytesRemaining(src) != 3) {
            status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        } else {
            startFieldId = sbufReadU16(src);
            maximumRecords = sbufReadU8(src);
            if (maximumRecords == 0) {
                status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
            }
        }
        mspGhostDpWriteCatalogResponse(dst, &request, status,
            startFieldId, maximumRecords);
        return MSP_RESULT_ACK;
    }

    default:
        return MSP_RESULT_ERROR;
    }
}

#endif // USE_MSP_DISPLAYPORT
