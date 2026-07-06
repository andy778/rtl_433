/** @file
    Uponor Clean 1 wastewater treatment unit - 868.2 MHz radio link.

    Copyright (C) 2026 Andy B

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**
Uponor Clean 1 - 868.2 MHz link between the outer (tank) unit and the indoor
info panel.

U1 is a Nordic nRF9E5 (nRF905 radio + 8051). The 8051 has no external boot
chip - its firmware is served from the MC9S08 (U2) flash at boot, so the whole
protocol was recovered by disassembling U2. Full derivation:
github.com/andy778/UClean1 (docs/rtl433-decoder.md).

Frame (confirmed from firmware and live captures):

    [preamble] [address EA EA EA EA, 4B] [payload 32B] [CRC-16 2B]

- 100 kbps, Manchester-coded on air (a PHY effect; neither firmware does
  Manchester in software).
- Channel 117 = 868.2 MHz.
- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over address+payload,
  big-endian trailer. Gates every decode.

The payload is a small messaging protocol, not analog telemetry - the panel has
only 5 alarm symbols + an OK/status LED, and that is all the radio carries:

    [0]         12 + N       frame length
    [1..2]      CC 6E        constant tag
    [3]         dir | flags  0x40 poll (outer->panel), 0x80 response (panel ACK)
    [4..6]      node_src     sender's 3-byte node ID
    [7..9]      node_dst     recipient's 3-byte node ID (echoed in the ACK)
    [10]        N            message-body length
    [11..10+N]  body         N=0 ACK, N=1 heartbeat (0x24), N=2 alarm [type,state]
    [11+N..31]  stale        leftover TX buffer, NOT data (varies run-to-run;
                             CRC still passes - U2 CRCs whatever it sends)

Alarm body = [type, state], state 0/1 = clear/active. The 5 types are the 5
panel symbols; 4 confirmed on air in a boot burst, all clear:

    0x20 status/OK   0x21 chemical_low   0x22 high_water
    0x23 sludge_reminder   0x26 device_fault   (0x24 = heartbeat, not an alarm)

Node IDs are stable per device (outer 80 0D 6E, panel C0 23 4B) - a poll/ack
cookie: each side sends its own and echoes the other's.

NOT on the radio: the cycle counter and treatment-phase code (firmware-confirmed;
a 20 h capture over a counter tick showed identical heartbeats). Read those over
the serial CODE interface instead (docs/u2-serial-protocol.md).

Flex-decoder equivalent (no C decoder):
  rtl_433 -f 868.20M -Y minmax \
    -X 'n=uclean1,m=FSK_MC_ZEROBIT,s=10,r=100,bits>=200,invert'
Tune ~150 kHz low so the carrier clears the RTL-SDR DC spike; -Y minmax is
required (the default detector fragments the weaker far-unit packet).
*/

#include "decoder.h"

// Raw line rate 100 kbps => 10 us/bit; the Manchester half-bit is 10 us, so the
// ZEROBIT slicer uses short == long == 10 us.
#define UCLEAN1_BIT_US 10

// Frame = [ADDRESS 4][PAYLOAD 32][CRC-16 2], anchored on the nRF905 address.
static uint8_t const uclean1_addr[] = {0xea, 0xea, 0xea, 0xea};

#define UCLEAN1_ADDR_LEN     4
#define UCLEAN1_PAYLOAD_LEN  32
#define UCLEAN1_CRC_LEN      2
#define UCLEAN1_FRAME_LEN    (UCLEAN1_ADDR_LEN + UCLEAN1_PAYLOAD_LEN + UCLEAN1_CRC_LEN) // 38
#define UCLEAN1_NODE_LEN     3

// Payload offsets (see the frame map in the file header).
#define UCLEAN1_OFF_DIR       3  // 0x40 poll / 0x80 response
#define UCLEAN1_OFF_NODE_SRC  4  // sender node ID, 3 bytes
#define UCLEAN1_OFF_NODE_DST  7  // recipient node ID, 3 bytes (echoed)
#define UCLEAN1_OFF_MSGLEN   10  // N = message-body length; frame[0] = 12 + N
#define UCLEAN1_OFF_MSGTYPE  11  // body[0] = type (0x20-0x26 alarm, 0x24 heartbeat)
#define UCLEAN1_OFF_MSGSTATE 12  // body[1] = state 0/1 (alarm frames, N >= 2)

static void bytes_to_hex(char *dst, uint8_t const *src, unsigned len)
{
    for (unsigned i = 0; i < len; ++i)
        snprintf(dst + i * 2, 3, "%02x", src[i]);
}

static int uponor_clean1_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    int ret = DECODE_ABORT_EARLY;

    for (unsigned row = 0; row < bitbuffer->num_rows; ++row) {
        unsigned row_bits = bitbuffer->bits_per_row[row];

        // FSK_PULSE_MANCHESTER_ZEROBIT already Manchester-decoded the row.
        if (row_bits < UCLEAN1_FRAME_LEN * 8) {
            ret = DECODE_ABORT_LENGTH;
            continue;
        }

        // The ZEROBIT slicer phase-locks on a leading zero bit, so the address
        // can land at any bit offset - bitbuffer_search finds it. Its output
        // polarity depends on which Manchester convention matched, so if the
        // anchor is not found, invert the whole buffer and try once more.
        unsigned pos = bitbuffer_search(bitbuffer, row, 0,
                uclean1_addr, sizeof(uclean1_addr) * 8);
        int inverted = 0;
        if (pos >= row_bits) {
            bitbuffer_invert(bitbuffer);
            inverted = 1;
            pos = bitbuffer_search(bitbuffer, row, 0,
                    uclean1_addr, sizeof(uclean1_addr) * 8);
        }
        if (pos >= row_bits) {
            if (inverted)
                bitbuffer_invert(bitbuffer); // restore for the next row
            decoder_log(decoder, 2, __func__, "address EAEAEAEA not found");
            ret = DECODE_ABORT_EARLY;
            continue;
        }
        if (pos + UCLEAN1_FRAME_LEN * 8 > row_bits) {
            if (inverted)
                bitbuffer_invert(bitbuffer);
            ret = DECODE_ABORT_LENGTH;
            continue;
        }

        uint8_t frame[UCLEAN1_FRAME_LEN] = {0};
        bitbuffer_extract_bytes(bitbuffer, row, pos, frame, UCLEAN1_FRAME_LEN * 8);

        if (inverted)
            bitbuffer_invert(bitbuffer); // restore for the next row

        // nRF905 hardware CRC-16 over ADDRESS+PAYLOAD. Gates the decode.
        uint16_t crc_calc = crc16(frame, UCLEAN1_ADDR_LEN + UCLEAN1_PAYLOAD_LEN, 0x1021, 0xffff);
        uint16_t crc_recv = (frame[UCLEAN1_ADDR_LEN + UCLEAN1_PAYLOAD_LEN] << 8)
                | frame[UCLEAN1_ADDR_LEN + UCLEAN1_PAYLOAD_LEN + 1];
        if (crc_calc != crc_recv) {
            decoder_log(decoder, 2, __func__, "CRC-16 mismatch");
            ret = DECODE_FAIL_MIC;
            continue;
        }

        uint8_t const *payload = &frame[UCLEAN1_ADDR_LEN];

        char id_str[UCLEAN1_ADDR_LEN * 2 + 1];
        bytes_to_hex(id_str, frame, UCLEAN1_ADDR_LEN);

        char node_src[UCLEAN1_NODE_LEN * 2 + 1];
        char node_dst[UCLEAN1_NODE_LEN * 2 + 1];
        bytes_to_hex(node_src, payload + UCLEAN1_OFF_NODE_SRC, UCLEAN1_NODE_LEN);
        bytes_to_hex(node_dst, payload + UCLEAN1_OFF_NODE_DST, UCLEAN1_NODE_LEN);

        char payload_str[UCLEAN1_PAYLOAD_LEN * 2 + 1];
        bytes_to_hex(payload_str, payload, UCLEAN1_PAYLOAD_LEN);

        int dir = payload[UCLEAN1_OFF_DIR];
        char const *role = (dir & 0x80) ? "response"
                         : (dir & 0x40) ? "poll"
                         : "?";

        // Message body: payload[10] = N, then N bytes at payload[11]. For alarm
        // frames (N >= 2) the body is [type, state]; type is one of the 5 panel
        // symbols.
        int msg_len   = payload[UCLEAN1_OFF_MSGLEN];
        int msg_type  = payload[UCLEAN1_OFF_MSGTYPE];  // valid when msg_len >= 1
        int msg_state = payload[UCLEAN1_OFF_MSGSTATE]; // valid when msg_len >= 2
        char const *msg_name =
                  msg_type == 0x20 ? "status"
                : msg_type == 0x21 ? "chemical_low"
                : msg_type == 0x22 ? "high_water"
                : msg_type == 0x23 ? "sludge_reminder"
                : msg_type == 0x26 ? "device_fault"
                : msg_type == 0x24 ? "heartbeat"
                : "?";

        /* clang-format off */
        data_t *data = data_make(
                "model",     "",           DATA_STRING, "Uponor-Clean-1",
                "id",        "Address",    DATA_STRING, id_str,
                "role",      "Role",       DATA_STRING, role,
                "node_src",  "Node src",   DATA_STRING, node_src,
                "node_dst",  "Node dst",   DATA_STRING, node_dst,
                "msg_len",   "Msg length", DATA_INT,    msg_len,
                "msg_type",  "Msg type",   DATA_COND, msg_len >= 1, DATA_FORMAT, "0x%02x", DATA_INT, msg_type,
                "msg_name",  "Msg name",   DATA_COND, msg_len >= 1, DATA_STRING, msg_name,
                "msg_state", "Msg state",  DATA_COND, msg_len >= 2, DATA_INT,    msg_state,
                "payload",   "Payload",    DATA_STRING, payload_str,
                "mic",       "Integrity",  DATA_STRING, "CRC",
                NULL);
        /* clang-format on */
        decoder_output_data(decoder, data);
        return 1;
    }

    return ret;
}

static char const *const uponor_clean1_output_fields[] = {
        "model",
        "id",
        "role",
        "node_src",
        "node_dst",
        "msg_len",
        "msg_type",
        "msg_name",
        "msg_state",
        "payload",
        "mic",
        NULL,
};

// reset_limit ends the packet after a gap with no transition; Manchester
// guarantees a transition at least every 2 half-bits, so ~100 us is safe and
// still splits the two packets of an exchange (~19 ms apart) into separate rows.
r_device const uponor_clean1 = {
        .name        = "Uponor Clean 1 wastewater unit (needs -f 868.2M)",
        .modulation  = FSK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = UCLEAN1_BIT_US,
        .long_width  = UCLEAN1_BIT_US,
        .reset_limit = 100,
        .decode_fn   = &uponor_clean1_decode,
        .fields      = uponor_clean1_output_fields,
};
