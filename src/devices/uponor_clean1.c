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
U1 on the PCB is a Nordic nRF9E5 (nRF905 transceiver + 8051 MCU). The radio PHY
is nRF905: GFSK, 868 MHz band. Everything below was measured from real RTL-SDR
captures (see docs/rtl433.md, "Empirical results"), NOT assumed from the
datasheet - and several datasheet-default assumptions turned out to be wrong.

CONFIRMED from captures
-----------------------
- Carrier   : ~868.3 MHz (a specific nRF905 channel).
- Modulation: FSK, deviation ~+-38 kHz.
- LINE RATE : 100 kbps (10 us/bit), NOT the nRF905 50 kbps default.
- CODING    : the payload is MANCHESTER-encoded (the nRF9E5 firmware does this
              in software - the radio itself is NRZ). Raw 100 kbps line =>
              ~50 kbps of Manchester data, ~40 bytes/packet. This is the single
              most important correction over the old skeleton, which assumed
              plain NRZ.
              Convention observed: raw "01" -> 1, raw "10" -> 0.
- FRAMING   : each transmit event (every ~62 s) is a TWO-packet exchange -
              packet A (~6.2 ms) then a ~19 ms gap then packet B (~6.6 ms), with
              very different RSSI = the near/far units answering each other.
- SYNC/HDR  : after a repeating 0xBA run, every Manchester-decoded packet begins
              with a constant header:  fd 7a  ba ba ba  83  <payload...>.
              The 0xfd 0x7a pair is used here as the frame anchor.
- PAYLOAD   : ~30 bytes after the header. Across consecutive events the payload
              is near-static (3 of 4 events were byte-identical), i.e. it is
              slowly-changing telemetry, not a counter/nonce.

STILL UNKNOWN (do not invent)
-----------------------------
  [U5] exact payload length / where the payload ends and any CRC begins.
  [U4] CRC: nRF905 hardware CRC covers the *on-air* (pre-Manchester) bytes, so it
       is not visible in the Manchester-decoded stream; there may additionally be
       an application checksum inside the payload. Unconfirmed - not gated on.
  [U6] field semantics: which decoded byte is water level / temperature / phase /
       alarm code / pump state. The MC9S08 firmware was checked (see docs/rtl433.md
       "0b. What the MC9S08 firmware says") and the payload byte layout is NOT in
       it: the CPU is only an SPI *slave* on a custom MC9S08<->8051 message bus, so
       the on-air framing and byte packing are done by the nRF9E5's 8051, not the
       CPU. The firmware does give the decode *targets* - the alarm code table
       (0x02 no-radio, 0x28 compressor, 0x29-0x2D MV1-5, 0x2F EEPROM error, ...)
       and the phase names, which the payload codes should index into. So [U6] must
       be resolved by a display-correlated capture: change the level / force an
       alarm and watch which decoded payload byte moves.

Because [U5]/[U6] are open, this decoder emits the constant header plus the raw
Manchester-decoded payload as a hex string, so captures can be correlated with
the display. It ships disabled until the fields and CRC are pinned down.

Flex-decoder equivalent for a first capture pass. Tune ~150 kHz low so the
carrier lands off the RTL-SDR DC spike; -Y minmax selects the FSK peak detector
(otherwise the weak far-unit packet fragments). Best option - let the flex
demodulator do the Manchester decode, so the payload comes out readable:
  rtl_433 -f 868.20M -Y minmax \
    -X 'n=uclean1,m=FSK_MC_ZEROBIT,s=10,r=100,bits>=200,invert'
FSK_MC_ZEROBIT is a demod-LEVEL Manchester slicer (decodes from pulse timing),
so s=10 is the half-bit period and the output is the DECODED bytes (~326 bits,
half of ~650). "invert" makes it match our raw 01->1 / 10->0 convention, so the
fd 7a ba ba ba 83 header prints verbatim. Full recall (all 8 packets of a 4-event
replay). Its phase lock assumes a leading zero bit, so a row may be off by one
bit - just search for fd7a as this decoder does. This C decoder uses the same
demod (FSK_PULSE_MANCHESTER_ZEROBIT), so the framework hands us the already
Manchester-decoded bytes and we simply anchor on fd7a.
NOTE the demod-level slicer is NOT the same as the "decode_mc" post-filter, which
aborts at the first Manchester violation and only tries phase 0, so on these
phase-offset, slightly-noisy packets it collapses to empty rows - that filter is
unusable here.
To instead see the RAW on-air bits (Manchester still encoded, decode by eye
01->1/10->0), use FSK_PCM at the full 100 kbps line rate:
  rtl_433 -f 868.20M -Y minmax -X 'n=uclean1,m=FSK_PCM,s=10,l=10,r=100,bits>=400'
*/

#include "decoder.h"

// Raw line rate 100 kbps => 10 us/bit; the Manchester half-bit is 10 us, so the
// ZEROBIT slicer uses short==long==10 us.
#define UCLEAN1_BIT_US 10

// Manchester-decoded header anchor (see notes above): fd 7a precedes the packet.
static uint8_t const uclean1_sync[] = {0xfd, 0x7a};

// Conservative bounds on the Manchester-decoded packet (bytes).
#define UCLEAN1_MIN_BYTES 20
#define UCLEAN1_MAX_BYTES 64

static int uponor_clean1_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    int ret = DECODE_ABORT_EARLY;

    for (unsigned row = 0; row < bitbuffer->num_rows; ++row) {
        unsigned row_bits = bitbuffer->bits_per_row[row];

        // FSK_PULSE_MANCHESTER_ZEROBIT already Manchester-decoded the row, so a
        // packet is header(6) + ~30 payload bytes of DECODED data.
        if (row_bits < UCLEAN1_MIN_BYTES * 8) {
            ret = DECODE_ABORT_LENGTH;
            continue;
        }

        // The ZEROBIT slicer phase-locks on a leading zero bit, so the fd 7a
        // anchor can land at any bit offset - search for it. Its output polarity
        // depends on which Manchester convention matched, so if the anchor is not
        // found, invert the whole buffer and try once more (then restore).
        unsigned pos = bitbuffer_search(bitbuffer, row, 0,
                uclean1_sync, sizeof(uclean1_sync) * 8);
        int inverted = 0;
        if (pos >= row_bits) {
            bitbuffer_invert(bitbuffer);
            inverted = 1;
            pos = bitbuffer_search(bitbuffer, row, 0,
                    uclean1_sync, sizeof(uclean1_sync) * 8);
        }
        if (pos >= row_bits) {
            if (inverted)
                bitbuffer_invert(bitbuffer); // restore for the next row
            decoder_log(decoder, 2, __func__, "sync fd7a not found");
            ret = DECODE_ABORT_EARLY;
            continue;
        }

        // Header = fd 7a ba ba ba 83  (6 bytes = 48 bits); payload follows.
        if (pos + (6 + UCLEAN1_MIN_BYTES) * 8 > row_bits) {
            if (inverted)
                bitbuffer_invert(bitbuffer);
            ret = DECODE_ABORT_LENGTH;
            continue;
        }

        // Byte-align header + payload from the anchor. Extract what we have.
        unsigned avail_bytes = (row_bits - pos) / 8;
        if (avail_bytes > UCLEAN1_MAX_BYTES)
            avail_bytes = UCLEAN1_MAX_BYTES;
        uint8_t frame[UCLEAN1_MAX_BYTES] = {0};
        bitbuffer_extract_bytes(bitbuffer, row, pos, frame, avail_bytes * 8);

        // frame[0..5] = header, frame[6..] = payload.
        unsigned payload_len = avail_bytes - 6;

        char header_str[6 * 2 + 1];
        for (unsigned i = 0; i < 6; ++i)
            snprintf(header_str + i * 2, 3, "%02x", frame[i]);

        char payload_str[UCLEAN1_MAX_BYTES * 2 + 1];
        for (unsigned i = 0; i < payload_len; ++i)
            snprintf(payload_str + i * 2, 3, "%02x", frame[6 + i]);

        /* clang-format off */
        data_t *data = data_make(
                "model",        "",             DATA_STRING, "Uponor-Clean-1",
                "header",       "Header",       DATA_STRING, header_str,
                "payload",      "Payload",      DATA_STRING, payload_str,
                "payload_len",  "Payload bytes",DATA_INT,    (int)payload_len,
                NULL);
        /* clang-format on */
        decoder_output_data(decoder, data);
        return 1;
    }

    return ret;
}

static char const *const uponor_clean1_output_fields[] = {
        "model",
        "header",
        "payload",
        "payload_len",
        // Real sensor fields go here once [U6] is resolved, e.g.:
        // "level_pct", "temperature_C", "phase", "alarm", "pump_on",
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
        .disabled    = 1,   // keep disabled until CRC + fields are validated
        .fields      = uponor_clean1_output_fields,
};
