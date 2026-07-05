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
- CODING    : the raw 100 kbps line Manchester-DECODES cleanly (~3-4% illegal
              pairs, vs ~50% for random NRZ) with convention raw "01" -> 1,
              "10" -> 0, giving ~50 kbps / ~40 bytes/packet. BUT see the caveat
              below: neither firmware image applies a software Manchester step,
              so whether this is true Manchester line-coding or an artifact of
              the byte patterns is now in question ([U5]).
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
  [U5] The on-air<->message transform is the key open question, and it just got
       more interesting. Tracing BOTH firmware images end to end:
       - MC9S08 side: the frame serializer FUN_ce01 appends message bytes RAW to
         a 128-byte ring buffer (FUN_8ec7: STA 0x0253,X, index 0x0251 & 0x7f) -
         no XOR/shift/expand, no Manchester.
       - nRF9E5 side: the 8051 relays those bytes verbatim into the nRF905
         W_TX_PAYLOAD (see docs/nrf9e5-firmware.md) - also no Manchester.
       So NEITHER firmware does a software Manchester step, yet the on-air line
       Manchester-decodes cleanly. That is not reconciled. Possibilities, none
       confirmed: (a) the nRF905 payload really is the raw 32 message bytes sent
       via ShockBurst (preamble + EAEAEAEA address + 32B payload + hw CRC-16),
       and the "Manchester" the flex decoder finds is a coincidence of the byte
       patterns; (b) a transform happens in the SPI hand-off path not yet traced;
       (c) the radio runs in a non-ShockBurst mode. The decisive test: take a RAW
       capture (FSK_PCM, pre-Manchester) and try to parse it as native nRF905
       ShockBurst (address EAEAEAEA, 32B payload) WITHOUT Manchester, then check
       the trailing bytes as the hw CRC-16 and the in-payload CRC-16/CCITT ([U4]).
       Do NOT assume the current Manchester model is correct until that is done.
  [U4] There are two CRC-16s, and the trailer IS a CRC after all:
       1. nRF905 hardware CRC-16 over the pre-Manchester on-air address+payload
          (config CRC_EN=1/CRC_MODE=1). Verified + stripped by the radio, so not
          in the decoded stream - nothing to check here.
       2. The 2-byte payload trailer that section "0d" of docs/rtl433.md could
          not match to any standard CRC. The MC9S08 firmware settles it: the
          frame serializer FUN_ce01 computes CRC-16/CCITT-FALSE (poly 0x1021,
          init 0xFFFF, MSB-first, no reflection, no xorout, transmitted
          big-endian) via a nibble-table update (FUN_e0cd -> FUN_e08e; the two
          16-entry tables at flash 0x8b14/0x8b24 match the 0x1021 nibble table
          exactly) over the MC9S08's *message* bytes, and appends it.
          It did NOT match in "0d" because that search ran over the
          Manchester-decoded ON-AIR bytes, which are a further-encoded form of
          the message - the covered byte range and the on-air<->message
          transform are the same open question as [U5]. So the algorithm is
          fully known; verifying/using it in this decoder needs the
          on-air->message byte mapping resolved first.
          (An earlier guess that the FUN_b218/b3d0/... chain computed this was
          WRONG - that chain is the LCD menu renderer.)
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
