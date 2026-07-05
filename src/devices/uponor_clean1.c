/** @file
    Uponor Clean 1 wastewater treatment unit - 868.35 MHz telemetry link.

    Copyright (C) 2026 Andy B

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**
Uponor Clean 1 - inner/outer unit radio link.

The Clean 1 links the outer (tank) unit and the inner (indoor display) unit.
U1 is a Nordic nRF9E5 (nRF905 transceiver + 8051 MCU). There is no external
boot chip for the 8051 - its firmware is embedded in the MC9S08 (U2) flash
(dumps/u2-mc9s08gt-flash.bin), served over SPI at boot, and was extracted +
disassembled (dumps/u1-nrf9e5-8051.bin). See project docs/nrf9e5-firmware.md
and docs/rtl433-decoder.md (github.com/andy778/UClean1) for the full derivation;
this comment covers only what a maintainer needs to trust the decoder.

Frame, confirmed both from the 8051's nRF905 W_CONFIG write and from a real
capture (tools/rtl433/probe_capture.py: Manchester-decodes, finds the address,
CRC-16 passes):

    [preamble] [address: EA EA EA EA, 4B] [payload: 32B] [CRC-16: 2B]

- Line rate 100 kbps, Manchester-coded on air (neither firmware applies
  Manchester in software - MC9S08 FUN_ce01/FUN_8ec7 and the 8051 relay both
  copy bytes raw, so this is a radio/PHY-layer effect).
- Channel 117 / 868.2 MHz (matches the measured carrier).
- CRC-16: poly 0x1021, init 0xFFFF, over address+payload, big-endian trailer.
  Verified live in this decoder (mic=CRC) and via probe_capture.py.
- The 8051 never computes payload content - it relays whatever the MC9S08
  sends it byte-for-byte (traced via the 8051's serial ISR at 0x029f).

Payload sub-fields (bytes 0-9 verified byte-exact against 2 real CRC-passing
packets by reading the MC9S08 frame serializer FUN_ce01's raw assembly):

    payload[0]    = record[8]+record[0xb]+12         (length/sub-type byte)
    payload[3]    = (record[8]&3)<<4|record[9]|0x0f  (candidate type/dir marker)
    payload[4]    = MC9S08 RAM 0x013e, low byte only (high byte never sent)
    payload[5:7]  = MC9S08 RAM 0x0140, full 16-bit
    payload[7]    = MC9S08 RAM 0x0109, low byte only
    payload[8:10] = MC9S08 RAM 0x010b, full 16-bit

Named by RAM origin, not a physical unit - meaning is unknown. Structurally
these look like a poll/ack correlation cookie, not sensor telemetry: the
MC9S08's response handler (FUN_caee) checks the incoming frame's matching
fields for equality before accepting a reply, and packet-pair captures show
(013e_lo + full 0140) in one packet swapping with (0109_lo + full 010b) in the
other - i.e. each side sends "my (013e,0140)" and echoes "your (0109,010b)".

Confirmed OFF the radio: CYCLE COUNTER (MC9S08 RAM 0x0607) and the PLANT
STATUS source (RAM 0x0613/0x0614) - neither is radio'd out (also confirmed
empirically: a 20 h capture spanning a real counter tick showed byte-identical
heartbeat frames). Get those via the serial CODE interface instead
(docs/u2-serial-protocol.md).

Payload bytes [10:32) are NOT mapped - the assembly traces to a count-loop
with computed/indirect addressing not fully resolved statically. Physical
field semantics (level/temperature/phase/alarm) need either more firmware
tracing or a display-correlated capture. The alarm code table
(docs/eeprom-map.md) and phase/status S-codes (docs/u2-serial-protocol.md)
are the decode targets once a byte offset is found.

Status: ENABLED. CRC-16 gates every frame; tested end-to-end in rtl_433 on
g001_868.2M_1000k.cu8 (both packets decode with mic=CRC) and an unrelated
433 MHz capture produces zero false positives.

Flex-decoder equivalent, for capturing without this C decoder:
  rtl_433 -f 868.20M -Y minmax \
    -X 'n=uclean1,m=FSK_MC_ZEROBIT,s=10,r=100,bits>=200,invert'
Tune ~150 kHz low so the carrier clears the RTL-SDR DC spike; -Y minmax is
required (the default detector fragments the weaker far-unit packet).
FSK_MC_ZEROBIT is a demodulator-level Manchester slicer (decodes from pulse
timing, unlike the decode_mc post-filter which aborts on the first violation
and does not work on these phase-offset packets). "invert" matches our raw
01->1/10->0 convention. For the raw (still Manchester-encoded) bit stream:
  rtl_433 -f 868.20M -Y minmax -X 'n=uclean1,m=FSK_PCM,s=10,l=10,r=100,bits>=400'
*/

#include "decoder.h"

// Raw line rate 100 kbps => 10 us/bit; the Manchester half-bit is 10 us, so the
// ZEROBIT slicer uses short==long==10 us.
#define UCLEAN1_BIT_US 10

// Frame anchor = the nRF905 4-byte address EA EA EA EA (from firmware; confirmed
// on-air, see the file header). In the Manchester-decoded stream the frame is
// [ADDRESS 4][PAYLOAD 32][CRC-16 2]; the CRC-16 (poly 0x1021, init 0xFFFF over
// ADDRESS+PAYLOAD) verifies on real captures, so it gates the decode.
static uint8_t const uclean1_addr[] = {0xea, 0xea, 0xea, 0xea};

#define UCLEAN1_ADDR_LEN     4
#define UCLEAN1_PAYLOAD_LEN  32
#define UCLEAN1_CRC_LEN      2
#define UCLEAN1_FRAME_LEN    (UCLEAN1_ADDR_LEN + UCLEAN1_PAYLOAD_LEN + UCLEAN1_CRC_LEN) // 38

// Sub-field offsets within the 32-byte payload, verified against real captures
// by reading the raw HCS08 assembly of the MC9S08 frame serializer FUN_ce01
// (see the file header, [U6]). Byte-exact match on 4 independent predictions
// across two differently-typed captured packets. Names are RAM-address-based
// (where the byte comes from in the firmware), NOT a physical unit - we do not
// yet know what these RAM words represent, only where they sit on the wire.
#define UCLEAN1_OFF_HDR0     0  // record[8]+record[0xb]+12 (length/sub-type)
#define UCLEAN1_OFF_HDR3     3  // (record[8]&3)<<4|record[9]|0x0f (candidate type/direction marker)
#define UCLEAN1_OFF_013E_LO  4  // _DAT_013e, low byte only (high byte never sent)
#define UCLEAN1_OFF_0140_HI  5  // _DAT_0140, high byte
#define UCLEAN1_OFF_0140_LO  6  // _DAT_0140, low byte
#define UCLEAN1_OFF_0109_LO  7  // _DAT_0109, low byte only (high byte never sent)
#define UCLEAN1_OFF_010B_HI  8  // _DAT_010b, high byte
#define UCLEAN1_OFF_010B_LO  9  // _DAT_010b, low byte
// Bytes [10, 32) are NOT yet mapped - see the file header [U6].

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
        // anchor can land at any bit offset - bitbuffer_search finds it. Its
        // output polarity depends on which Manchester convention matched, so if
        // the anchor is not found, invert the whole buffer and try once more.
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

        // nRF905 hardware CRC-16 over ADDRESS+PAYLOAD (poly 0x1021, init 0xFFFF),
        // trailing big-endian. Gates the decode.
        uint16_t crc_calc = crc16(frame, UCLEAN1_ADDR_LEN + UCLEAN1_PAYLOAD_LEN, 0x1021, 0xffff);
        uint16_t crc_recv = (frame[UCLEAN1_ADDR_LEN + UCLEAN1_PAYLOAD_LEN] << 8)
                | frame[UCLEAN1_ADDR_LEN + UCLEAN1_PAYLOAD_LEN + 1];
        if (crc_calc != crc_recv) {
            decoder_log(decoder, 2, __func__, "CRC-16 mismatch");
            ret = DECODE_FAIL_MIC;
            continue;
        }

        char id_str[UCLEAN1_ADDR_LEN * 2 + 1];
        for (unsigned i = 0; i < UCLEAN1_ADDR_LEN; ++i)
            snprintf(id_str + i * 2, 3, "%02x", frame[i]);

        // Payload byte semantics ([U6]) are only partly mapped, so emit the raw
        // CRC-validated 32-byte payload as hex for correlation with the display,
        // ALONGSIDE the sub-fields whose byte position is now verified (see the
        // offsets above). The sub-fields are named after their MC9S08 RAM origin,
        // not a physical unit - their real-world meaning is still unknown.
        char payload_str[UCLEAN1_PAYLOAD_LEN * 2 + 1];
        for (unsigned i = 0; i < UCLEAN1_PAYLOAD_LEN; ++i)
            snprintf(payload_str + i * 2, 3, "%02x", frame[UCLEAN1_ADDR_LEN + i]);

        uint8_t const *payload   = &frame[UCLEAN1_ADDR_LEN];
        int hdr0                 = payload[UCLEAN1_OFF_HDR0];
        int hdr3                 = payload[UCLEAN1_OFF_HDR3];
        int word_013e_lo         = payload[UCLEAN1_OFF_013E_LO];
        int word_0140            = (payload[UCLEAN1_OFF_0140_HI] << 8) | payload[UCLEAN1_OFF_0140_LO];
        int word_0109_lo         = payload[UCLEAN1_OFF_0109_LO];
        int word_010b            = (payload[UCLEAN1_OFF_010B_HI] << 8) | payload[UCLEAN1_OFF_010B_LO];

        /* clang-format off */
        data_t *data = data_make(
                "model",        "",             DATA_STRING, "Uponor-Clean-1",
                "id",           "Address",      DATA_STRING, id_str,
                "hdr0",         "Header byte 0", DATA_FORMAT, "0x%02x", DATA_INT, hdr0,
                "hdr3",         "Header byte 3", DATA_FORMAT, "0x%02x", DATA_INT, hdr3,
                "word_013e_lo", "RAM 0x013e (low byte)", DATA_INT, word_013e_lo,
                "word_0140",    "RAM 0x0140",   DATA_INT,   word_0140,
                "word_0109_lo", "RAM 0x0109 (low byte)", DATA_INT, word_0109_lo,
                "word_010b",    "RAM 0x010b",   DATA_INT,   word_010b,
                "payload",      "Payload",      DATA_STRING, payload_str,
                "mic",          "Integrity",    DATA_STRING, "CRC",
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
        "hdr0",
        "hdr3",
        "word_013e_lo",
        "word_0140",
        "word_0109_lo",
        "word_010b",
        "payload",
        "mic",
        // Real sensor fields (level/temperature/phase/alarm) go here once the
        // physical meaning of the word_* fields above, and the unmapped tail
        // (payload bytes 10-31), are resolved - see [U6] in the file header.
        NULL,
};

// 100 kbps line rate => 10 us Manchester half-bit. FSK_PULSE_MANCHESTER_ZEROBIT
// slices on that half-bit period (short==long==10 us) and hands back the decoded
// data bits directly. reset_limit ends the packet after a gap with no
// transition; Manchester guarantees a transition at least every 2 half-bits, so
// ~100 us is safe and still splits the two packets of an exchange (~19 ms apart)
// into separate rows.
r_device const uponor_clean1 = {
        .name        = "Uponor Clean 1 wastewater unit (needs -f 868.2M)",
        .modulation  = FSK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = UCLEAN1_BIT_US,
        .long_width  = UCLEAN1_BIT_US,
        .reset_limit = 100,
        .decode_fn   = &uponor_clean1_decode,
        // CRC-16 now gates every frame (address + hw CRC verified on real
        // captures), so enabled. Payload field semantics ([U6]) are still raw
        // hex, but the CRC prevents false positives.
        .disabled    = 0,
        .fields      = uponor_clean1_output_fields,
};
