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

#ifdef USE_MSP_GHOST_DP

#include "common/time.h"
#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "io/displayport_msp_osd.h"
#include "mavlink/mavlink_mission.h"
#include "navigation/navigation.h"

#ifdef USE_GPS
#include "io/gps.h"
#endif

#include "msp/msp_ghost_dp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

typedef struct mspGhostDpHeader_s {
    uint8_t version;
    uint8_t messageType;
    uint8_t flags;
    uint8_t source;
    uint8_t destination;
    uint16_t sessionId;
    uint16_t exchangeId;
} mspGhostDpHeader_t;

typedef struct mspGhostDpRelayMailbox_s {
    uint16_t exchangeId;
    uint16_t length;
    bool waiting;
    bool ready;
    uint8_t payload[MSP_PORT_INBUF_SIZE];
} mspGhostDpRelayMailbox_t;

static mspGhostDpRelayMailbox_t ghostRelayMailbox;

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

enum {
    MSP_GHOST_DP_MAX_SLOTS = 22,
    MSP_GHOST_DP_MAX_STREAM_BPS = 115200,
    MSP_GHOST_DP_DEFAULT_LEASE_SECONDS = 5,
    MSP_GHOST_DP_MIN_LEASE_SECONDS = 2,
    MSP_GHOST_DP_MAX_LEASE_SECONDS = 30,
    MSP_GHOST_DP_QUOTE_LIFETIME_MS = 5000,
};

typedef struct mspGhostDpQuoteEntry_s {
    uint8_t requestIndex;
    uint8_t instance;
    uint8_t priority;
    uint8_t requestFlags;
    uint8_t deadbandRaw;
    mspGhostDpStatus_e status;
    uint16_t offeredRateHz;
    const mspGhostDpFieldDescriptor_t *field;
} mspGhostDpQuoteEntry_t;

typedef struct mspGhostDpQuote_s {
    bool valid;
    uint16_t sessionId;
    uint16_t requestRevision;
    uint32_t token;
    uint32_t expiresAtMs;
    uint32_t estimatedBps;
    uint8_t leaseSeconds;
    uint8_t entryCount;
    mspGhostDpStatus_e status;
    mspGhostDpQuoteEntry_t entries[MSP_GHOST_DP_MAX_SLOTS];
} mspGhostDpQuote_t;

typedef struct mspGhostDpStreamEntry_s {
    uint8_t slot;
    uint8_t instance;
    uint16_t effectiveRateHz;
    uint32_t nextDueUs;
    uint32_t lastSentUs;
    uint8_t lastValue[8];
    uint8_t lastSize;
    uint8_t lastFlags;
    uint8_t deadbandRaw;
    bool hasLastValue;
    const mspGhostDpFieldDescriptor_t *field;
} mspGhostDpStreamEntry_t;

typedef struct mspGhostDpStream_s {
    bool active;
    uint16_t generation;
    uint32_t expiresAtMs;
    uint32_t effectiveBps;
    uint32_t committedToken;
    uint16_t committedRevision;
    uint8_t leaseSeconds;
    uint8_t entryCount;
    mspGhostDpStreamEntry_t entries[MSP_GHOST_DP_MAX_SLOTS];
} mspGhostDpStream_t;

typedef struct mspGhostDpMutationCache_s {
    bool valid;
    uint8_t messageType;
    uint16_t sessionId;
    uint16_t exchangeId;
    mspGhostDpStatus_e status;
    uint16_t generation;
    uint8_t leaseSeconds;
    uint32_t effectiveBps;
} mspGhostDpMutationCache_t;

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
    { 16, MSP_GHOST_DP_VALUE_I16, MSP_GHOST_DP_UNIT_DEGREES_PER_SECOND, -1,
      FIELD_SIGNED, 100, 100, 1, "ANGULAR_RATE_ROLL" },
    { 17, MSP_GHOST_DP_VALUE_I16, MSP_GHOST_DP_UNIT_DEGREES_PER_SECOND, -1,
      FIELD_SIGNED, 100, 100, 1, "ANGULAR_RATE_PITCH" },
    { 18, MSP_GHOST_DP_VALUE_I16, MSP_GHOST_DP_UNIT_DEGREES_PER_SECOND, -1,
      FIELD_SIGNED, 100, 100, 1, "ANGULAR_RATE_YAW" },
    { 22, MSP_GHOST_DP_VALUE_U16, MSP_GHOST_DP_UNIT_VOLT, -2,
      FIELD_INVALID, 20, 10, 1, "BATTERY_CELL_VOLTAGE" },
    { 23, MSP_GHOST_DP_VALUE_U8, MSP_GHOST_DP_UNIT_COUNT, 0,
      FIELD_INVALID, 10, 1, 1, "BATTERY_CELL_COUNT" },
    { 24, MSP_GHOST_DP_VALUE_U32, MSP_GHOST_DP_UNIT_WATT_HOUR, -3,
      FIELD_INVALID, 20, 10, 1, "BATTERY_WH" },
    { 25, MSP_GHOST_DP_VALUE_U8, MSP_GHOST_DP_UNIT_PERCENT, 0,
      FIELD_INVALID, 10, 10, 1, "BATTERY_REMAINING_PERCENT" },
    { 26, MSP_GHOST_DP_VALUE_U8, MSP_GHOST_DP_UNIT_NONE, 0,
      0, 10, 10, 1, "BATTERY_STATE" },
#ifdef USE_GPS
    { 27, MSP_GHOST_DP_VALUE_I32, MSP_GHOST_DP_UNIT_DEGREE, -7,
      FIELD_SIGNED | FIELD_INVALID, 10, 10, 1, "HOME_LATITUDE" },
    { 28, MSP_GHOST_DP_VALUE_I32, MSP_GHOST_DP_UNIT_DEGREE, -7,
      FIELD_SIGNED | FIELD_INVALID, 10, 10, 1, "HOME_LONGITUDE" },
#endif
    { 29, MSP_GHOST_DP_VALUE_U8, MSP_GHOST_DP_UNIT_COUNT, 0,
      FIELD_INVALID, 10, 10, 1, "MISSION_ACTIVE_WAYPOINT" },
    { 30, MSP_GHOST_DP_VALUE_U8, MSP_GHOST_DP_UNIT_NONE, 0,
      0, 10, 10, 1, "MISSION_STATE" },
    { 31, MSP_GHOST_DP_VALUE_U8, MSP_GHOST_DP_UNIT_NONE, 0,
      0, 10, 10, 1, "MISSION_ABORT_REASON" },
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
    { 50, MSP_GHOST_DP_VALUE_BOOL, MSP_GHOST_DP_UNIT_NONE, 0,
      0, 10, 10, 1, "MISSION_ACTIVE" },
};

#undef RC_FIELD
#undef FIELD_INVALID
#undef FIELD_SIGNED

static uint32_t ghostBootId;
static uint16_t ghostSessionId;
static uint32_t ghostCatalogHash;
static mspGhostDpQuote_t ghostQuote;
static mspGhostDpStream_t ghostStream;
static mspGhostDpMutationCache_t ghostMutationCache;
static bool ghostStreamMapPending;
static uint16_t ghostStreamMapExchangeId;
static uint16_t ghostPushExchangeId;

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

static const mspGhostDpFieldDescriptor_t *mspGhostDpFindField(uint16_t fieldId)
{
    for (unsigned index = 0; index < sizeof(fieldCatalog) / sizeof(fieldCatalog[0]); ++index) {
        if (fieldCatalog[index].id == fieldId) {
            return &fieldCatalog[index];
        }
    }
    return NULL;
}

static uint8_t mspGhostDpValueSize(uint8_t valueType)
{
    switch (valueType) {
    case MSP_GHOST_DP_VALUE_U8:
    case MSP_GHOST_DP_VALUE_I8:
    case MSP_GHOST_DP_VALUE_BOOL:
        return 1;
    case MSP_GHOST_DP_VALUE_U16:
    case MSP_GHOST_DP_VALUE_I16:
        return 2;
    case MSP_GHOST_DP_VALUE_U32:
    case MSP_GHOST_DP_VALUE_I32:
    case MSP_GHOST_DP_VALUE_F32:
        return 4;
    case MSP_GHOST_DP_VALUE_U64:
    case MSP_GHOST_DP_VALUE_I64:
    case MSP_GHOST_DP_VALUE_F64:
        return 8;
    default:
        return 0;
    }
}

static uint32_t mspGhostDpEstimateFieldBps(
    const mspGhostDpFieldDescriptor_t *field, uint16_t rateHz)
{
    /*
     * Conservative upper bound: one complete native MSPv2 FIELD_DATA packet
     * per field update, without relying on future batching.
     */
    const uint32_t bytesPerPacket = 9 + MSP_GHOST_DP_HEADER_SIZE + 3 + 3 +
        mspGhostDpValueSize(field->valueType);
    return bytesPerPacket * 8u * rateHz;
}

static uint8_t mspGhostDpClampLease(uint8_t requested)
{
    if (requested == 0) {
        return MSP_GHOST_DP_DEFAULT_LEASE_SECONDS;
    }
    if (requested < MSP_GHOST_DP_MIN_LEASE_SECONDS) {
        return MSP_GHOST_DP_MIN_LEASE_SECONDS;
    }
    if (requested > MSP_GHOST_DP_MAX_LEASE_SECONDS) {
        return MSP_GHOST_DP_MAX_LEASE_SECONDS;
    }
    return requested;
}

static uint16_t mspGhostDpNextGeneration(void)
{
    ++ghostStream.generation;
    if (ghostStream.generation == 0) {
        ghostStream.generation = 1;
    }
    return ghostStream.generation;
}

static uint16_t mspGhostDpNextPushExchange(void)
{
    ++ghostPushExchangeId;
    if (ghostPushExchangeId == 0) {
        ghostPushExchangeId = 1;
    }
    return ghostPushExchangeId;
}

static void mspGhostDpScheduleStreamMap(uint16_t exchangeId)
{
    ghostStreamMapPending = true;
    ghostStreamMapExchangeId = exchangeId != 0 ? exchangeId :
        mspGhostDpNextPushExchange();
}

static void mspGhostDpExpireStream(uint32_t nowMs)
{
    if (!ghostStream.active || cmp32(nowMs, ghostStream.expiresAtMs) < 0) {
        return;
    }
    ghostStream.active = false;
    ghostStream.entryCount = 0;
    ghostStream.leaseSeconds = 0;
    ghostStream.effectiveBps = 0;
    ghostStream.committedToken = 0;
    ghostStream.committedRevision = 0;
    ghostMutationCache.valid = false;
    mspGhostDpNextGeneration();
    mspGhostDpScheduleStreamMap(0);
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
    uint32_t value = ticks() ^ U_ID_0;
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
    sbufWriteU32(dst, (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4) |
        (1u << 5) | (1u << 10) | (1u << 11) | (1u << 12));
    sbufWriteU16(dst, MSP_PORT_INBUF_SIZE);  // max_payload
    sbufWriteU32(dst, MSP_GHOST_DP_MAX_STREAM_BPS);
    sbufWriteU8(dst, MSP_GHOST_DP_MAX_SLOTS);
    sbufWriteU8(dst, MSP_GHOST_DP_DEFAULT_LEASE_SECONDS);
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

static uint32_t mspGhostDpQuoteToken(const mspGhostDpQuote_t *quote,
    uint16_t exchangeId)
{
    uint32_t hash = 2166136261u;
    hash = fnv1aByte(hash, ghostSessionId);
    hash = fnv1aByte(hash, ghostSessionId >> 8);
    hash = fnv1aByte(hash, quote->requestRevision);
    hash = fnv1aByte(hash, quote->requestRevision >> 8);
    hash = fnv1aByte(hash, exchangeId);
    hash = fnv1aByte(hash, exchangeId >> 8);
    hash = fnv1aByte(hash, quote->leaseSeconds);
    for (unsigned index = 0; index < quote->entryCount; ++index) {
        const mspGhostDpQuoteEntry_t *entry = &quote->entries[index];
        hash = fnv1aByte(hash, entry->requestIndex);
        hash = fnv1aByte(hash, entry->field ? entry->field->id : 0);
        hash = fnv1aByte(hash, entry->field ? entry->field->id >> 8 : 0);
        hash = fnv1aByte(hash, entry->instance);
        hash = fnv1aByte(hash, entry->offeredRateHz);
        hash = fnv1aByte(hash, entry->offeredRateHz >> 8);
        hash = fnv1aByte(hash, entry->deadbandRaw);
        hash = fnv1aByte(hash, entry->status);
    }
    return hash != 0 ? hash : 1;
}

static bool mspGhostDpQuoteHasDuplicate(unsigned currentIndex)
{
    const mspGhostDpQuoteEntry_t *current = &ghostQuote.entries[currentIndex];
    if (!current->field) {
        return false;
    }
    for (unsigned index = 0; index < currentIndex; ++index) {
        const mspGhostDpQuoteEntry_t *other = &ghostQuote.entries[index];
        if (other->field == current->field && other->instance == current->instance) {
            return true;
        }
    }
    return false;
}

static bool mspGhostDpQuoteEntryAccepted(const mspGhostDpQuoteEntry_t *entry)
{
    return (entry->status == MSP_GHOST_DP_STATUS_OK ||
            entry->status == MSP_GHOST_DP_STATUS_RATE_LIMITED) &&
        entry->offeredRateHz != 0;
}

static void mspGhostDpAdmitQuoteEntries(bool required)
{
    bool considered[MSP_GHOST_DP_MAX_SLOTS] = { false };
    for (unsigned count = 0; count < ghostQuote.entryCount; ++count) {
        int selected = -1;
        for (unsigned index = 0; index < ghostQuote.entryCount; ++index) {
            const mspGhostDpQuoteEntry_t *entry = &ghostQuote.entries[index];
            const bool isRequired = (entry->requestFlags & 1u) != 0;
            if (considered[index] || isRequired != required ||
                !mspGhostDpQuoteEntryAccepted(entry)) {
                continue;
            }
            if (selected < 0 ||
                entry->priority < ghostQuote.entries[selected].priority) {
                selected = index;
            }
        }
        if (selected < 0) {
            break;
        }
        considered[selected] = true;
        mspGhostDpQuoteEntry_t *entry = &ghostQuote.entries[selected];
        const uint32_t entryBps = mspGhostDpEstimateFieldBps(
            entry->field, entry->offeredRateHz);
        if (entryBps > MSP_GHOST_DP_MAX_STREAM_BPS - ghostQuote.estimatedBps) {
            entry->status = MSP_GHOST_DP_STATUS_BANDWIDTH_EXCEEDED;
            entry->offeredRateHz = 0;
            if (required && ghostQuote.status == MSP_GHOST_DP_STATUS_OK) {
                ghostQuote.status = MSP_GHOST_DP_STATUS_BANDWIDTH_EXCEEDED;
            }
        } else {
            ghostQuote.estimatedBps += entryBps;
        }
    }
}

static void mspGhostDpReadQuote(sbuf_t *src, const mspGhostDpHeader_t *request,
    mspGhostDpStatus_e initialStatus)
{
    memset(&ghostQuote, 0, sizeof(ghostQuote));
    ghostQuote.sessionId = ghostSessionId;
    ghostQuote.status = initialStatus;

    if (initialStatus != MSP_GHOST_DP_STATUS_OK) {
        return;
    }
    if (sbufBytesRemaining(src) < 4) {
        ghostQuote.status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        return;
    }

    ghostQuote.requestRevision = sbufReadU16(src);
    ghostQuote.leaseSeconds = mspGhostDpClampLease(sbufReadU8(src));
    ghostQuote.entryCount = sbufReadU8(src);
    if (ghostQuote.entryCount == 0) {
        ghostQuote.entryCount = 0;
        ghostQuote.status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        return;
    }
    if (ghostQuote.entryCount > MSP_GHOST_DP_MAX_SLOTS) {
        ghostQuote.entryCount = 0;
        ghostQuote.status = MSP_GHOST_DP_STATUS_TOO_MANY_FIELDS;
        return;
    }
    for (unsigned index = 0; index < ghostQuote.entryCount; ++index) {
        if (sbufBytesRemaining(src) < 10) {
            ghostQuote.entryCount = 0;
            ghostQuote.status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
            return;
        }
        mspGhostDpQuoteEntry_t *entry = &ghostQuote.entries[index];
        entry->requestIndex = sbufReadU8(src);
        const uint16_t fieldId = sbufReadU16(src);
        entry->instance = sbufReadU8(src);
        const uint16_t minimumRateHz = sbufReadU16(src);
        const uint16_t preferredRateHz = sbufReadU16(src);
        entry->priority = sbufReadU8(src);
        entry->requestFlags = sbufReadU8(src);
        if (entry->requestFlags & MSP_GHOST_DP_SUBSCRIPTION_HAS_DEADBAND) {
            if (sbufBytesRemaining(src) < 1) {
                ghostQuote.entryCount = 0;
                ghostQuote.status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
                return;
            }
            entry->deadbandRaw = sbufReadU8(src);
        }
        entry->field = mspGhostDpFindField(fieldId);
        entry->status = MSP_GHOST_DP_STATUS_OK;

        const bool required = (entry->requestFlags & 1u) != 0;
        const bool optional = (entry->requestFlags & 2u) != 0;
        if ((entry->requestFlags & ~(MSP_GHOST_DP_SUBSCRIPTION_REQUIRED |
              MSP_GHOST_DP_SUBSCRIPTION_OPTIONAL |
              MSP_GHOST_DP_SUBSCRIPTION_HAS_DEADBAND)) || required == optional) {
            entry->status = MSP_GHOST_DP_STATUS_INVALID_TRANSACTION;
        } else if (!entry->field) {
            entry->status = MSP_GHOST_DP_STATUS_UNSUPPORTED_FIELD;
        } else if (entry->instance != 0) {
            entry->status = MSP_GHOST_DP_STATUS_INVALID_INSTANCE;
        } else if (minimumRateHz == 0 || preferredRateHz < minimumRateHz ||
                   minimumRateHz > entry->field->maximumRateHz) {
            entry->status = MSP_GHOST_DP_STATUS_INVALID_RATE;
        } else {
            entry->offeredRateHz = preferredRateHz;
            if (entry->offeredRateHz > entry->field->maximumRateHz) {
                entry->offeredRateHz = entry->field->maximumRateHz;
                entry->status = MSP_GHOST_DP_STATUS_RATE_LIMITED;
            }
        }

        if (mspGhostDpQuoteHasDuplicate(index)) {
            entry->status = MSP_GHOST_DP_STATUS_INVALID_TRANSACTION;
            entry->offeredRateHz = 0;
        }
        if (entry->status != MSP_GHOST_DP_STATUS_OK &&
            entry->status != MSP_GHOST_DP_STATUS_RATE_LIMITED && required &&
            ghostQuote.status == MSP_GHOST_DP_STATUS_OK) {
            ghostQuote.status = entry->status;
        }
    }

    if (sbufBytesRemaining(src) != 0) {
        ghostQuote.entryCount = 0;
        ghostQuote.status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        return;
    }

    mspGhostDpAdmitQuoteEntries(true);
    mspGhostDpAdmitQuoteEntries(false);

    if (ghostQuote.status == MSP_GHOST_DP_STATUS_OK) {
        ghostQuote.token = mspGhostDpQuoteToken(&ghostQuote,
            request->exchangeId);
        ghostQuote.expiresAtMs = millis() + MSP_GHOST_DP_QUOTE_LIFETIME_MS;
        ghostQuote.valid = true;
    }
}

static void mspGhostDpWriteQuoteResponse(sbuf_t *dst,
    const mspGhostDpHeader_t *request)
{
    mspGhostDpWriteHeader(dst, MSP_GHOST_DP_SUBSCRIPTION_QUOTE_RESULT,
        responseFlags(ghostQuote.status), request->source, ghostSessionId,
        request->exchangeId);
    sbufWriteU8(dst, ghostQuote.status);
    sbufWriteU16(dst, ghostQuote.requestRevision);
    sbufWriteU32(dst, ghostQuote.valid ? ghostQuote.token : 0);
    sbufWriteU32(dst, ghostQuote.estimatedBps);
    sbufWriteU8(dst, ghostQuote.leaseSeconds);
    sbufWriteU8(dst, ghostQuote.entryCount);
    for (unsigned index = 0; index < ghostQuote.entryCount; ++index) {
        const mspGhostDpQuoteEntry_t *entry = &ghostQuote.entries[index];
        sbufWriteU8(dst, entry->requestIndex);
        sbufWriteU8(dst, entry->status);
        sbufWriteU16(dst, entry->offeredRateHz);
    }
}

static void mspGhostDpWriteSubscriptionResult(sbuf_t *dst,
    const mspGhostDpHeader_t *request, mspGhostDpStatus_e status,
    uint16_t generation, uint8_t leaseSeconds, uint32_t effectiveBps)
{
    mspGhostDpWriteHeader(dst, MSP_GHOST_DP_SUBSCRIPTION_RESULT,
        responseFlags(status), request->source, ghostSessionId,
        request->exchangeId);
    sbufWriteU8(dst, status);
    sbufWriteU16(dst, generation);
    sbufWriteU8(dst, leaseSeconds);
    sbufWriteU32(dst, effectiveBps);
}

static bool mspGhostDpWriteCachedMutation(sbuf_t *dst,
    const mspGhostDpHeader_t *request)
{
    if (!ghostMutationCache.valid ||
        ghostMutationCache.messageType != request->messageType ||
        ghostMutationCache.sessionId != request->sessionId ||
        ghostMutationCache.exchangeId != request->exchangeId) {
        return false;
    }
    mspGhostDpWriteSubscriptionResult(dst, request, ghostMutationCache.status,
        ghostMutationCache.generation, ghostMutationCache.leaseSeconds,
        ghostMutationCache.effectiveBps);
    if (ghostMutationCache.status == MSP_GHOST_DP_STATUS_OK &&
        (request->messageType == MSP_GHOST_DP_SUBSCRIPTION_COMMIT ||
         request->messageType == MSP_GHOST_DP_SUBSCRIPTION_RELEASE)) {
        mspGhostDpScheduleStreamMap(request->exchangeId);
    }
    return true;
}

static void mspGhostDpCacheMutation(const mspGhostDpHeader_t *request,
    mspGhostDpStatus_e status, uint16_t generation, uint8_t leaseSeconds,
    uint32_t effectiveBps)
{
    ghostMutationCache = (mspGhostDpMutationCache_t) {
        .valid = true,
        .messageType = request->messageType,
        .sessionId = request->sessionId,
        .exchangeId = request->exchangeId,
        .status = status,
        .generation = generation,
        .leaseSeconds = leaseSeconds,
        .effectiveBps = effectiveBps,
    };
}

static void mspGhostDpApplyQuote(uint32_t nowMs)
{
    const uint32_t nowUs = micros();
    ghostStream.active = true;
    ghostStream.entryCount = 0;
    ghostStream.leaseSeconds = ghostQuote.leaseSeconds;
    ghostStream.effectiveBps = ghostQuote.estimatedBps;
    ghostStream.committedToken = ghostQuote.token;
    ghostStream.committedRevision = ghostQuote.requestRevision;
    ghostStream.expiresAtMs = nowMs + ghostStream.leaseSeconds * 1000u;
    mspGhostDpNextGeneration();
    for (unsigned index = 0; index < ghostQuote.entryCount; ++index) {
        const mspGhostDpQuoteEntry_t *quoted = &ghostQuote.entries[index];
        if (!mspGhostDpQuoteEntryAccepted(quoted)) {
            continue;
        }
        mspGhostDpStreamEntry_t *stream =
            &ghostStream.entries[ghostStream.entryCount];
        stream->slot = ghostStream.entryCount;
        stream->instance = quoted->instance;
        stream->effectiveRateHz = quoted->offeredRateHz;
        stream->nextDueUs = nowUs;
        stream->deadbandRaw = quoted->deadbandRaw;
        stream->field = quoted->field;
        ++ghostStream.entryCount;
    }
    ghostQuote.valid = false;
}

static int mspGhostDpPushStreamMap(void)
{
    uint8_t payload[MSP_GHOST_DP_HEADER_SIZE + 5 +
        MSP_GHOST_DP_MAX_SLOTS * 9];
    sbuf_t dst = { .ptr = payload, .end = payload + sizeof(payload) };
    mspGhostDpWriteHeader(&dst, MSP_GHOST_DP_STREAM_MAP, 0,
        MSP_GHOST_DP_ENDPOINT_VRX, ghostSessionId, ghostStreamMapExchangeId);
    sbufWriteU16(&dst, ghostStream.generation);
    sbufWriteU8(&dst, 0); // fragment_index
    sbufWriteU8(&dst, 1); // fragment_count
    sbufWriteU8(&dst, ghostStream.entryCount);
    for (unsigned index = 0; index < ghostStream.entryCount; ++index) {
        const mspGhostDpStreamEntry_t *entry = &ghostStream.entries[index];
        sbufWriteU8(&dst, entry->slot);
        sbufWriteU16(&dst, entry->field->id);
        sbufWriteU8(&dst, entry->instance);
        sbufWriteU8(&dst, entry->field->valueType);
        sbufWriteU8(&dst, entry->field->unit);
        sbufWriteU8(&dst, (uint8_t)entry->field->scaleExponent);
        sbufWriteU16(&dst, entry->effectiveRateHz);
    }
    const int payloadLength = sbufPtr(&dst) - payload;
    return mspSerialPushVersion(MSP_DISPLAYPORT, payload, payloadLength,
        MSP_V2_NATIVE);
}

enum {
    MSP_GHOST_DP_VALUE_VALID = 1 << 0,
};

static uint8_t mspGhostDpSampleField(const mspGhostDpStreamEntry_t *entry,
    uint8_t *value, uint8_t *valueFlags)
{
    bool valid = true;
    uint32_t rawValue = 0;

    switch (entry->field->id) {
    case 1: // PITCH, decidegrees
        rawValue = (uint16_t)attitude.values.pitch;
        valid = sensors(SENSOR_ACC);
        break;
    case 2: // ROLL, decidegrees
        rawValue = (uint16_t)attitude.values.roll;
        valid = sensors(SENSOR_ACC);
        break;
    case 3: // HEADING, decidegrees
        rawValue = (uint16_t)attitude.values.yaw;
        break;
#ifdef USE_GPS
    case 4: // LATITUDE, degrees * 1e-7
        rawValue = (uint32_t)gpsSol.llh.lat;
        valid = STATE(GPS_FIX);
        break;
    case 5: // LONGITUDE, degrees * 1e-7
        rawValue = (uint32_t)gpsSol.llh.lon;
        valid = STATE(GPS_FIX);
        break;
    case 6: // GPS_ALTITUDE, centimetres
        rawValue = (uint32_t)gpsSol.llh.alt;
        valid = STATE(GPS_FIX);
        break;
    case 7: // GROUND_SPEED, centimetres per second
        rawValue = gpsSol.groundSpeed;
        valid = STATE(GPS_FIX);
        break;
#endif
    case 8: // BATTERY_VOLTAGE, millivolts
        rawValue = getBatteryVoltage() * 10u;
        valid = getBatteryState() != BATTERY_NOT_PRESENT;
        break;
    case 9: // BATTERY_CURRENT, centiamperes
        rawValue = (uint32_t)getAmperage();
        valid = isAmperageConfigured();
        break;
    case 10: // BATTERY_MAH, milliampere-hours
        rawValue = (uint32_t)getMAhDrawn();
        valid = isAmperageConfigured();
        break;
#ifdef USE_GPS
    case 11: // GPS_SATELLITES
        rawValue = gpsSol.numSat;
        valid = sensors(SENSOR_GPS);
        break;
    case 12: // HOME_BEARING, decidegrees
        rawValue = (uint16_t)GPS_directionToHome;
        valid = STATE(GPS_FIX_HOME);
        break;
#endif
    case 13: // HEADING_VALID
        rawValue = sensors(SENSOR_MAG) ? 1u : 0u;
        break;
#ifdef USE_GPS
    case 14: // GPS_FIX
        rawValue = STATE(GPS_FIX) ? 1u : 0u;
        break;
    case 15: // HOME_VALID
        rawValue = STATE(GPS_FIX_HOME) ? 1u : 0u;
        break;
#endif
    case 16: // ANGULAR_RATE_ROLL, decidegrees per second
        rawValue = (uint16_t)(gyroRateDps(X) * 10);
        break;
    case 17: // ANGULAR_RATE_PITCH, decidegrees per second
        rawValue = (uint16_t)(gyroRateDps(Y) * 10);
        break;
    case 18: // ANGULAR_RATE_YAW, decidegrees per second
        rawValue = (uint16_t)(gyroRateDps(Z) * 10);
        break;
    case 22: // BATTERY_CELL_VOLTAGE, centivolts
        rawValue = getBatteryAverageCellVoltage() / 10u;
        valid = getBatteryState() != BATTERY_NOT_PRESENT && getBatteryCellCount() > 0;
        break;
    case 23: // BATTERY_CELL_COUNT
        rawValue = getBatteryCellCount();
        valid = getBatteryState() != BATTERY_NOT_PRESENT;
        break;
    case 24: // BATTERY_WH, milliwatt-hours
        rawValue = (uint32_t)getMWhDrawn();
        valid = isAmperageConfigured();
        break;
    case 25: // BATTERY_REMAINING_PERCENT
        rawValue = calculateBatteryPercentage();
        valid = getBatteryState() != BATTERY_NOT_PRESENT;
        break;
    case 26: // BATTERY_STATE
        rawValue = getBatteryState();
        break;
#ifdef USE_GPS
    case 27: // HOME_LATITUDE
        rawValue = (uint32_t)GPS_home.lat;
        valid = STATE(GPS_FIX_HOME);
        break;
    case 28: // HOME_LONGITUDE
        rawValue = (uint32_t)GPS_home.lon;
        valid = STATE(GPS_FIX_HOME);
        break;
#endif
    case 29: // MISSION_ACTIVE_WAYPOINT
        rawValue = getActiveWpNumber();
        valid = isWaypointListValid();
        break;
    case 30: // MISSION_STATE
        rawValue = FLIGHT_MODE(NAV_WP_MODE) ? 1u : 0u;
        break;
    case 31: // MISSION_ABORT_REASON (not currently exposed by INAV)
        rawValue = 0;
        break;
    case 50: // MISSION_ACTIVE
        rawValue = FLIGHT_MODE(NAV_WP_MODE) ? 1u : 0u;
        break;
    default:
        if (entry->field->id >= 32 && entry->field->id <= 49) {
            const unsigned channel = entry->field->id - 32;
            valid = channel < rxRuntimeConfig.channelCount;
            rawValue = valid ? (uint16_t)rxGetChannelValue(channel) : 0;
        } else {
            valid = false;
        }
        break;
    }

    *valueFlags = valid ? MSP_GHOST_DP_VALUE_VALID : 0;
    const uint8_t valueSize = mspGhostDpValueSize(entry->field->valueType);
    for (unsigned index = 0; index < valueSize; ++index) {
        value[index] = rawValue >> (index * 8);
    }
    return valueSize;
}

static int64_t mspGhostDpIntegerValue(uint8_t type, const uint8_t *value)
{
    uint64_t raw = 0;
    const uint8_t size = mspGhostDpValueSize(type);
    for (uint8_t index = 0; index < size; ++index) {
        raw |= (uint64_t)value[index] << (index * 8u);
    }
    switch (type) {
    case MSP_GHOST_DP_VALUE_I8:
        return (int8_t)raw;
    case MSP_GHOST_DP_VALUE_I16:
        return (int16_t)raw;
    case MSP_GHOST_DP_VALUE_I32:
        return (int32_t)raw;
    case MSP_GHOST_DP_VALUE_I64:
        return (int64_t)raw;
    default:
        return (int64_t)raw;
    }
}

static bool mspGhostDpChangedByDeadband(const mspGhostDpStreamEntry_t *entry,
    const uint8_t *value, uint8_t size, uint8_t flags)
{
    if (!entry->hasLastValue || entry->deadbandRaw == 0 ||
        size != entry->lastSize || flags != entry->lastFlags) {
        return true;
    }
    const int64_t current = mspGhostDpIntegerValue(entry->field->valueType, value);
    const int64_t previous = mspGhostDpIntegerValue(entry->field->valueType,
        entry->lastValue);
    const uint64_t difference = current >= previous ?
        (uint64_t)(current - previous) : (uint64_t)(previous - current);
    return difference >= entry->deadbandRaw;
}

static int mspGhostDpPushFieldData(uint32_t nowUs)
{
    uint8_t payload[MSP_GHOST_DP_NEGOTIATION_PAYLOAD_MAX];
    sbuf_t dst = { .ptr = payload, .end = payload + sizeof(payload) };
    bool due[MSP_GHOST_DP_MAX_SLOTS] = { false };
    bool evaluated[MSP_GHOST_DP_MAX_SLOTS] = { false };
    uint8_t values[MSP_GHOST_DP_MAX_SLOTS][8] = { { 0 } };
    uint8_t valueSizes[MSP_GHOST_DP_MAX_SLOTS] = { 0 };
    uint8_t valueFlags[MSP_GHOST_DP_MAX_SLOTS] = { 0 };
    uint8_t dueCount = 0;

    for (unsigned index = 0; index < ghostStream.entryCount; ++index) {
        mspGhostDpStreamEntry_t *entry = &ghostStream.entries[index];
        if (cmp32(nowUs, entry->nextDueUs) < 0) {
            continue;
        }
        evaluated[index] = true;
        valueSizes[index] = mspGhostDpSampleField(entry, values[index],
            &valueFlags[index]);
        const bool keepaliveDue = entry->hasLastValue &&
            nowUs - entry->lastSentUs >= 1000000u;
        if (valueSizes[index] > 0 && (keepaliveDue ||
            mspGhostDpChangedByDeadband(entry, values[index], valueSizes[index],
                valueFlags[index]))) {
            due[index] = true;
            ++dueCount;
        }
    }

    for (unsigned index = 0; index < ghostStream.entryCount; ++index) {
        if (!evaluated[index]) {
            continue;
        }
        mspGhostDpStreamEntry_t *entry = &ghostStream.entries[index];
        const uint32_t intervalUs = 1000000u / entry->effectiveRateHz;
        const uint32_t elapsedUs = nowUs - entry->nextDueUs;
        entry->nextDueUs += (elapsedUs / intervalUs + 1u) * intervalUs;
    }
    if (dueCount == 0) {
        return 0;
    }

    mspGhostDpWriteHeader(&dst, MSP_GHOST_DP_FIELD_DATA, 0,
        MSP_GHOST_DP_ENDPOINT_VRX, ghostSessionId, mspGhostDpNextPushExchange());
    sbufWriteU16(&dst, ghostStream.generation);
    sbufWriteU8(&dst, dueCount);
    for (unsigned index = 0; index < ghostStream.entryCount; ++index) {
        if (!due[index]) {
            continue;
        }
        const mspGhostDpStreamEntry_t *entry = &ghostStream.entries[index];
        sbufWriteU8(&dst, entry->slot);
        sbufWriteU8(&dst, valueFlags[index]);
        sbufWriteU8(&dst, valueSizes[index]);
        sbufWriteData(&dst, values[index], valueSizes[index]);
    }

    const int payloadLength = sbufPtr(&dst) - payload;
    const int written = mspSerialPushVersion(MSP_DISPLAYPORT, payload,
        payloadLength, MSP_V2_NATIVE);
    if (written <= 0) {
        return written;
    }

    for (unsigned index = 0; index < ghostStream.entryCount; ++index) {
        if (!due[index]) {
            continue;
        }
        mspGhostDpStreamEntry_t *entry = &ghostStream.entries[index];
        memcpy(entry->lastValue, values[index], valueSizes[index]);
        entry->lastSize = valueSizes[index];
        entry->lastFlags = valueFlags[index];
        entry->lastSentUs = nowUs;
        entry->hasLastValue = true;
    }
    return written;
}

static bool mspGhostDpHeaderIsRequest(const mspGhostDpHeader_t *request)
{
    return (request->flags & MSP_GHOST_DP_FLAG_REQUEST) &&
        !(request->flags & MSP_GHOST_DP_FLAG_RESPONSE) &&
        (request->destination == MSP_GHOST_DP_ENDPOINT_FLIGHT_CONTROLLER ||
         request->destination == MSP_GHOST_DP_ENDPOINT_BROADCAST);
}

bool mspGhostDpProcessReply(sbuf_t *src)
{
    const unsigned payloadLength = sbufBytesRemaining(src);
    const uint8_t *const payload = sbufConstPtr(src);
    if (payloadLength < MSP_GHOST_DP_HEADER_SIZE ||
        payload[0] != MSP_DP_GHOST ||
        !(payload[3] & MSP_GHOST_DP_FLAG_RESPONSE) ||
        payload[4] != MSP_GHOST_DP_ENDPOINT_VRX ||
        payload[5] != MSP_GHOST_DP_ENDPOINT_CONFIGURATOR) {
        return false;
    }

    const uint16_t exchangeId = payload[8] | (payload[9] << 8);
    if (ghostRelayMailbox.waiting &&
        exchangeId == ghostRelayMailbox.exchangeId &&
        payloadLength <= sizeof(ghostRelayMailbox.payload)) {
        memcpy(ghostRelayMailbox.payload, payload, payloadLength);
        ghostRelayMailbox.length = payloadLength;
        ghostRelayMailbox.ready = true;
        ghostRelayMailbox.waiting = false;
    }
    return true;
}

static uint32_t mspGhostDpMissionHash(void)
{
    uint32_t hash = 2166136261u;
    const uint16_t count = getWaypointCount();
    hash = fnv1aByte(hash, count);
    hash = fnv1aByte(hash, count >> 8);
    for (uint16_t sequence = 0; sequence < count; ++sequence) {
        navWaypoint_t waypoint;
        getWaypoint(sequence + 1, &waypoint);
        const uint8_t *bytes = (const uint8_t *)&waypoint;
        for (unsigned index = 0; index < sizeof(waypoint); ++index) {
            hash = fnv1aByte(hash, bytes[index]);
        }
    }
    return hash != 0 ? hash : 1;
}

static void mspGhostDpWriteFloat(sbuf_t *dst, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    sbufWriteU32(dst, raw);
}

mspResult_e mspGhostDpProcessCommand(sbuf_t *src, sbuf_t *dst)
{
    const unsigned payloadLength = sbufBytesRemaining(src);
    const uint8_t *const payload = sbufConstPtr(src);
    if (payloadLength == 0 || sbufReadU8(src) != MSP_DP_GHOST) {
        return MSP_RESULT_ERROR;
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
    if ((request.flags & MSP_GHOST_DP_FLAG_RESPONSE) &&
        request.source == MSP_GHOST_DP_ENDPOINT_VRX &&
        request.destination == MSP_GHOST_DP_ENDPOINT_CONFIGURATOR) {
        if (ghostRelayMailbox.waiting &&
            request.exchangeId == ghostRelayMailbox.exchangeId &&
            payloadLength <= sizeof(ghostRelayMailbox.payload)) {
            memcpy(ghostRelayMailbox.payload, payload, payloadLength);
            ghostRelayMailbox.length = payloadLength;
            ghostRelayMailbox.ready = true;
            ghostRelayMailbox.waiting = false;
        }
        return MSP_RESULT_NO_REPLY;
    }
    if ((request.flags & MSP_GHOST_DP_FLAG_REQUEST) &&
        request.source == MSP_GHOST_DP_ENDPOINT_CONFIGURATOR &&
        request.destination == MSP_GHOST_DP_ENDPOINT_VRX) {
        mspPort_t *const osdPort = getMspOsdPort();
        if (payloadLength > MSP_PORT_INBUF_SIZE || !osdPort) {
            return MSP_RESULT_ERROR;
        }
        ghostRelayMailbox.exchangeId = request.exchangeId;
        ghostRelayMailbox.length = 0;
        ghostRelayMailbox.waiting = true;
        ghostRelayMailbox.ready = false;
        if (mspSerialPushPort(MSP_DISPLAYPORT, payload, payloadLength,
                osdPort, MSP_V2_NATIVE) <= 0) {
            ghostRelayMailbox.waiting = false;
            return MSP_RESULT_ERROR;
        }
        sbufWriteU16(dst, request.exchangeId);
        return MSP_RESULT_ACK;
    }
    if (request.messageType == MSP_GHOST_DP_RELAY_POLL &&
        request.source == MSP_GHOST_DP_ENDPOINT_CONFIGURATOR &&
        request.destination == MSP_GHOST_DP_ENDPOINT_FLIGHT_CONTROLLER &&
        (request.flags & MSP_GHOST_DP_FLAG_REQUEST) &&
        sbufBytesRemaining(src) == 2) {
        const uint16_t exchangeId = sbufReadU16(src);
        if (ghostRelayMailbox.ready &&
            exchangeId == ghostRelayMailbox.exchangeId) {
            sbufWriteData(dst, ghostRelayMailbox.payload,
                ghostRelayMailbox.length);
            ghostRelayMailbox.ready = false;
        } else {
            mspGhostDpWriteHeader(dst, MSP_GHOST_DP_RELAY_POLL_RESULT,
                responseFlags(MSP_GHOST_DP_STATUS_BUSY),
                MSP_GHOST_DP_ENDPOINT_CONFIGURATOR, request.sessionId,
                request.exchangeId);
            sbufWriteU16(dst, exchangeId);
        }
        return MSP_RESULT_ACK;
    }
    if (!mspGhostDpHeaderIsRequest(&request)) {
        return MSP_RESULT_ERROR;
    }

    const bool versionSupported =
        (request.version >> 4) == (MSP_GHOST_DP_VERSION_1_0 >> 4);
    mspGhostDpExpireStream(millis());
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

    case MSP_GHOST_DP_SUBSCRIPTION_QUOTE: {
        mspGhostDpStatus_e status = MSP_GHOST_DP_STATUS_OK;
        if (!versionSupported) {
            status = MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION;
        } else if (ghostSessionId == 0 || request.sessionId != ghostSessionId) {
            status = MSP_GHOST_DP_STATUS_INVALID_SESSION;
        } else if (!(request.flags & MSP_GHOST_DP_FLAG_VOLATILE)) {
            status = MSP_GHOST_DP_STATUS_INVALID_TRANSACTION;
        }
        mspGhostDpReadQuote(src, &request, status);
        mspGhostDpWriteQuoteResponse(dst, &request);
        return MSP_RESULT_ACK;
    }

    case MSP_GHOST_DP_SUBSCRIPTION_COMMIT: {
        if (mspGhostDpWriteCachedMutation(dst, &request)) {
            return MSP_RESULT_ACK;
        }
        mspGhostDpStatus_e status = MSP_GHOST_DP_STATUS_OK;
        uint16_t requestRevision = 0;
        uint32_t quoteToken = 0;
        const uint32_t nowMs = millis();
        if (!versionSupported) {
            status = MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION;
        } else if (ghostSessionId == 0 || request.sessionId != ghostSessionId) {
            status = MSP_GHOST_DP_STATUS_INVALID_SESSION;
        } else if (!(request.flags & MSP_GHOST_DP_FLAG_VOLATILE)) {
            status = MSP_GHOST_DP_STATUS_INVALID_TRANSACTION;
        } else if (sbufBytesRemaining(src) != 6) {
            status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        } else {
            requestRevision = sbufReadU16(src);
            quoteToken = sbufReadU32(src);
            if (!ghostQuote.valid || cmp32(nowMs, ghostQuote.expiresAtMs) >= 0 ||
                ghostQuote.sessionId != request.sessionId ||
                ghostQuote.requestRevision != requestRevision ||
                ghostQuote.token != quoteToken) {
                status = MSP_GHOST_DP_STATUS_INVALID_TRANSACTION;
            }
        }
        if (status == MSP_GHOST_DP_STATUS_OK) {
            mspGhostDpApplyQuote(nowMs);
            mspGhostDpScheduleStreamMap(request.exchangeId);
        }
        mspGhostDpWriteSubscriptionResult(dst, &request, status,
            ghostStream.generation,
            status == MSP_GHOST_DP_STATUS_OK ? ghostStream.leaseSeconds : 0,
            status == MSP_GHOST_DP_STATUS_OK ? ghostStream.effectiveBps : 0);
        mspGhostDpCacheMutation(&request, status, ghostStream.generation,
            status == MSP_GHOST_DP_STATUS_OK ? ghostStream.leaseSeconds : 0,
            status == MSP_GHOST_DP_STATUS_OK ? ghostStream.effectiveBps : 0);
        return MSP_RESULT_ACK;
    }

    case MSP_GHOST_DP_SUBSCRIPTION_RENEW: {
        if (mspGhostDpWriteCachedMutation(dst, &request)) {
            return MSP_RESULT_ACK;
        }
        mspGhostDpStatus_e status = MSP_GHOST_DP_STATUS_OK;
        uint16_t generation = 0;
        if (!versionSupported) {
            status = MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION;
        } else if (ghostSessionId == 0 || request.sessionId != ghostSessionId) {
            status = MSP_GHOST_DP_STATUS_INVALID_SESSION;
        } else if (!(request.flags & MSP_GHOST_DP_FLAG_VOLATILE)) {
            status = MSP_GHOST_DP_STATUS_INVALID_TRANSACTION;
        } else if (sbufBytesRemaining(src) != 2) {
            status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        } else {
            generation = sbufReadU16(src);
            if (!ghostStream.active || generation != ghostStream.generation) {
                status = MSP_GHOST_DP_STATUS_INVALID_TRANSACTION;
            }
        }
        if (status == MSP_GHOST_DP_STATUS_OK) {
            ghostStream.expiresAtMs = millis() + ghostStream.leaseSeconds * 1000u;
        }
        mspGhostDpWriteSubscriptionResult(dst, &request, status,
            ghostStream.generation,
            status == MSP_GHOST_DP_STATUS_OK ? ghostStream.leaseSeconds : 0,
            status == MSP_GHOST_DP_STATUS_OK ? ghostStream.effectiveBps : 0);
        mspGhostDpCacheMutation(&request, status, ghostStream.generation,
            status == MSP_GHOST_DP_STATUS_OK ? ghostStream.leaseSeconds : 0,
            status == MSP_GHOST_DP_STATUS_OK ? ghostStream.effectiveBps : 0);
        return MSP_RESULT_ACK;
    }

    case MSP_GHOST_DP_SUBSCRIPTION_RELEASE: {
        if (mspGhostDpWriteCachedMutation(dst, &request)) {
            return MSP_RESULT_ACK;
        }
        mspGhostDpStatus_e status = MSP_GHOST_DP_STATUS_OK;
        uint16_t generation = 0;
        if (!versionSupported) {
            status = MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION;
        } else if (ghostSessionId == 0 || request.sessionId != ghostSessionId) {
            status = MSP_GHOST_DP_STATUS_INVALID_SESSION;
        } else if (!(request.flags & MSP_GHOST_DP_FLAG_VOLATILE)) {
            status = MSP_GHOST_DP_STATUS_INVALID_TRANSACTION;
        } else if (sbufBytesRemaining(src) != 2) {
            status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        } else {
            generation = sbufReadU16(src);
            if (!ghostStream.active || generation != ghostStream.generation) {
                status = MSP_GHOST_DP_STATUS_INVALID_TRANSACTION;
            }
        }
        if (status == MSP_GHOST_DP_STATUS_OK) {
            ghostStream.active = false;
            ghostStream.entryCount = 0;
            ghostStream.leaseSeconds = 0;
            ghostStream.effectiveBps = 0;
            ghostStream.committedToken = 0;
            ghostStream.committedRevision = 0;
            mspGhostDpNextGeneration();
            mspGhostDpScheduleStreamMap(request.exchangeId);
        }
        mspGhostDpWriteSubscriptionResult(dst, &request, status,
            ghostStream.generation, 0, 0);
        mspGhostDpCacheMutation(&request, status, ghostStream.generation, 0, 0);
        return MSP_RESULT_ACK;
    }

    case MSP_GHOST_DP_MISSION_INFO_REQUEST: {
        uint8_t missionType = 0;
        mspGhostDpStatus_e status = MSP_GHOST_DP_STATUS_OK;
        if (!versionSupported) {
            status = MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION;
        } else if (ghostSessionId == 0 || request.sessionId != ghostSessionId) {
            status = MSP_GHOST_DP_STATUS_INVALID_SESSION;
        } else if (sbufBytesRemaining(src) != 1) {
            status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        } else {
            missionType = sbufReadU8(src);
            if (missionType != 0) {
                status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
            }
        }
        const uint32_t hash = mspGhostDpMissionHash();
        mspGhostDpWriteHeader(dst, MSP_GHOST_DP_MISSION_INFO_RESPONSE,
            responseFlags(status), request.source, ghostSessionId,
            request.exchangeId);
        sbufWriteU8(dst, status);
        sbufWriteU8(dst, missionType);
        sbufWriteU16(dst, status == MSP_GHOST_DP_STATUS_OK ? getWaypointCount() : 0);
        sbufWriteU16(dst, status == MSP_GHOST_DP_STATUS_OK ? NAV_MAX_WAYPOINTS : 0);
        sbufWriteU32(dst, status == MSP_GHOST_DP_STATUS_OK ? hash : 0);
        sbufWriteU32(dst, status == MSP_GHOST_DP_STATUS_OK ? hash : 0);
        return MSP_RESULT_ACK;
    }

    case MSP_GHOST_DP_MISSION_ITEM_REQUEST: {
        uint8_t missionType = 0;
        uint16_t sequence = 0;
        mspGhostDpStatus_e status = MSP_GHOST_DP_STATUS_OK;
        if (!versionSupported) {
            status = MSP_GHOST_DP_STATUS_UNSUPPORTED_VERSION;
        } else if (ghostSessionId == 0 || request.sessionId != ghostSessionId) {
            status = MSP_GHOST_DP_STATUS_INVALID_SESSION;
        } else if (sbufBytesRemaining(src) != 3) {
            status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
        } else {
            missionType = sbufReadU8(src);
            sequence = sbufReadU16(src);
            if (missionType != 0) {
                status = MSP_GHOST_DP_STATUS_BAD_LENGTH;
            }
        }
        navWaypoint_t waypoint = { 0 };
        mavlinkMissionItemData_t item = { 0 };
        if (status == MSP_GHOST_DP_STATUS_OK) {
            if (sequence >= getWaypointCount()) {
                status = MSP_GHOST_DP_STATUS_INVALID_MISSION;
            } else {
                getWaypoint(sequence + 1, &waypoint);
                if (!mavlinkFillMissionItemFromWaypoint(&waypoint, true, &item)) {
                    status = MSP_GHOST_DP_STATUS_INVALID_MISSION;
                }
            }
        }
        const uint32_t hash = mspGhostDpMissionHash();
        mspGhostDpWriteHeader(dst, MSP_GHOST_DP_MISSION_ITEM_RESPONSE,
            responseFlags(status), request.source, ghostSessionId,
            request.exchangeId);
        sbufWriteU8(dst, status);
        sbufWriteU8(dst, missionType);
        sbufWriteU32(dst, status == MSP_GHOST_DP_STATUS_OK ? hash : 0);
        sbufWriteU16(dst, sequence);
        if (status == MSP_GHOST_DP_STATUS_OK) {
            sbufWriteU8(dst, item.frame);
            sbufWriteU16(dst, item.command);
            sbufWriteU8(dst, getActiveWpNumber() == sequence + 1);
            sbufWriteU8(dst, 1); // INAV missions auto-continue.
            mspGhostDpWriteFloat(dst, item.param1);
            mspGhostDpWriteFloat(dst, item.param2);
            mspGhostDpWriteFloat(dst, item.param3);
            mspGhostDpWriteFloat(dst, item.param4);
            sbufWriteU32(dst, (uint32_t)item.lat);
            sbufWriteU32(dst, (uint32_t)item.lon);
            mspGhostDpWriteFloat(dst, item.alt);
        }
        return MSP_RESULT_ACK;
    }

    default:
        return MSP_RESULT_ERROR;
    }
}

void mspGhostDpProcess(void)
{
    mspGhostDpExpireStream(millis());
    if (ghostStreamMapPending) {
        if (mspGhostDpPushStreamMap() > 0) {
            ghostStreamMapPending = false;
        }
        return;
    }
    if (ghostStream.active) {
        mspGhostDpPushFieldData(micros());
    }
}

#endif // USE_MSP_GHOST_DP
