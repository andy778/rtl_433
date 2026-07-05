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
is nRF905: GFSK, 868 MHz band.

Two independent evidence sources agree here, which is why the numbers below are
trusted: (a) real RTL-SDR captures (see docs/rtl433.md, "Empirical results"), and
(b) the nRF9E5's own 8051 firmware. That firmware is NOT on a separate chip -
there is no boot EEPROM on this board. U1's SPI lines run to U2 (the MC9S08),
which serves the 8051 its boot image, so the 8051 program is embedded in
dumps/u2-mc9s08gt-flash.bin (see docs/nrf9e5-firmware.md) and was extracted +
disassembled (dumps/u1-nrf9e5-8051.bin).

CONFIRMED from captures
-----------------------
- Modulation: FSK, deviation ~+-38 kHz.
- LINE RATE : 100 kbps (10 us/bit), NOT the nRF905 50 kbps default.
- CODING    : the payload is MANCHESTER-encoded. Raw 100 kbps line => ~50 kbps
              of Manchester data, ~40 bytes/packet.
              Convention observed: raw "01" -> 1, raw "10" -> 0.
- FRAMING   : each transmit event (every ~62 s) is a TWO-packet exchange -
              packet A (~6.2 ms) then a ~19 ms gap then packet B (~6.6 ms), with
              very different RSSI = the near/far units answering each other.
- SYNC/HDR  : after a repeating 0xBA run, every Manchester-decoded packet begins
              with a constant header:  fd 7a  ba ba ba  83  <payload...>.
              The 0xfd 0x7a pair is used here as the frame anchor.
- PAYLOAD   : ~30 bytes after the header. Across consecutive events the payload
              is near-static (3 of 4 events were byte-identical), i.e. it is
              slowly-changing telemetry, not a counter/nonce. A ~1.5 h idle log
              additionally showed the response frame is 100% static and the poll
              frame is a doubled 13-byte record (see docs/rtl433.md "0d").

CONFIRMED from the 8051 firmware (W_CONFIG write, routine 0x015c in the
extracted image; see docs/nrf9e5-firmware.md)
-----------------------------------------------------------------------
- CHANNEL   : CH_NO=117, HFREQ_PLL=1 -> f = 868.2 MHz. Matches the measured
              carrier - an independent cross-check that this is the right image.
- ADDRESS   : 4 bytes, RX and TX address both EAEAEAEA (W_TX_ADDRESS).
- PAYLOAD   : nRF905 RX_PW/TX_PW = 32 bytes (hardware payload width register).
- CRC       : CRC_EN=1, CRC_MODE=1 -> 16-bit, computed by the nRF905 hardware
              over the pre-Manchester on-air bits. It is verified/stripped by
              the radio itself and is NOT present in the Manchester-decoded
              stream handed to this decoder - there is nothing to check here.
              (A separate, non-CRC application trailer inside the payload is
              still unresolved - see [U4] below.)
- RELAY     : the 8051 does not compute payload content. Tracing its serial ISR
              (0x029f) shows the MC9S08 sends a length byte (0x01-0x1f) + that
              many payload bytes, which the 8051 copies verbatim into the radio
              TX buffer (RAM 0x30-0x4F) and transmits; inbound data is relayed
              back the same way. So every payload byte is defined by the
              MC9S08 firmware, not the 8051.

STILL UNKNOWN (do not invent)
-----------------------------
  [U5] Reconcile the 32-byte nRF905 payload width with what's actually observed:
       captures show ~33 bytes after the 6-byte "fd7a bababa 83" header (see
       docs/rtl433.md "0d"). Whether that header **is** the 4-byte address
       (EAEAEAEA) at some bit offset, residual preamble, or something else has
       NOT been checked bit-for-bit - do not assume either way.
  [U4] Confirmed: nRF905 hardware CRC-16 exists but is not in the decoded stream
       (see above). Separately, section "0d" in docs/rtl433.md found the
       payload's 2-byte trailer is content-dependent but matches NO standard
       CRC-8/16/sum/Fletcher/Adler under exhaustive search - still open, likely
       a proprietary/nonlinear check computed on the MC9S08 side (candidate
       code: the B-prefixed routine chain reached from FUN_a77b, i.e.
       FUN_b218/FUN_b3d0/FUN_b328/FUN_b378/FUN_b865, which takes RAM 0x0109/
       0x010b as inputs - not yet decompiled/verified).
  [U6] field semantics: which decoded byte is water level / temperature / phase /
       alarm code / pump state. Two things are now confirmed from the MC9S08
       side that narrow this down (see docs/rtl433.md "0b"):
       - The satsraknare (CYCLE COUNTER) lives in MC9S08 RAM 0x0607, and the
         PLANT STATUS S-code is built from RAM 0x0613/0x0614 - NEITHER is among
         the four 16-bit words (RAM 0x013e/0x0140/0x0109/0x010b) that get
         radio'd out via FUN_db22/FUN_de1b. This matches the empirical finding
         that the counter never appears on air (a ~20 h capture spanning a real
         counter tick showed byte-identical heartbeat frames) - don't look for
         it in the radio payload.
       - Those four RAM words behave structurally like a poll/ack correlation
         pair, not raw sensor telemetry: FUN_caee (the response handler) checks
         the INCOMING frame's corresponding fields for equality against them
         (_DAT_0109==_DAT_0619, _DAT_010b==_DAT_061b, _DAT_013e==_DAT_061d,
         _DAT_0140==_DAT_061f) before accepting the reply. That is consistent
         with the empirically-observed "swapped 3-byte node ID" fields between
         poll and response (docs/rtl433.md "0c") being a correlation cookie
         rather than a physical measurement - though the exact RAM-word-to-
         payload-byte-offset mapping is not proven (the byte order the 8051
         relay actually transmits in has not been traced end to end).
       The alarm code table (0x02 no-radio, 0x28 compressor, 0x29-0x2D MV1-5,
       0x2F EEPROM error, ...) and the phase/status S-codes (see docs/eeprom-map.md
       and docs/u2-serial-protocol.md) remain the best decode *targets* once a
       byte offset is found. [U6] is still best resolved by a display-correlated
       capture: change the level / force an alarm and watch which decoded
       payload byte moves.

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

// Conservative bounds on the Manchester-decoded packet (bytes). The firmware's
// nRF905 config says the true payload is 32 bytes (see the file header), but
// that is not yet reconciled with the observed ~33-36 bytes after this 6-byte
// header ([U5]), so the bounds here stay loose rather than hard-coding 32.
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
