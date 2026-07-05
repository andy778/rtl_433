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
- CRC       : CRC_EN=1, CRC_MODE=1 -> 16-bit (poly 0x1021, init 0xFFFF, over
              ADDRESS+PAYLOAD). CONFIRMED on-air, not just from the datasheet:
              tools/rtl433/probe_capture.py FM-discriminates a real capture,
              Manchester-decodes it, finds the firmware address EA EA EA EA, and
              the trailing CRC-16 VERIFIES (g001_868.2M_1000k.cu8, burst1 ->
              CRC-16 PASS). So the frame is [preamble][EA EA EA EA][32B payload]
              [CRC-16], and the address / 32-byte width / CRC are all confirmed
              from the air. (A separate in-payload application check may also
              exist - the MC9S08 CRC-16/CCITT of [U4] - but it was not needed to
              validate the frame.)
- RELAY     : the 8051 does not compute payload content. Tracing its serial ISR
              (0x029f) shows the MC9S08 sends a length byte (0x01-0x1f) + that
              many payload bytes, which the 8051 copies verbatim into the radio
              TX buffer (RAM 0x30-0x4F) and transmits; inbound data is relayed
              back the same way. So every payload byte is defined by the
              MC9S08 firmware, not the 8051.

STILL UNKNOWN (do not invent)
-----------------------------
  [U5] RESOLVED (frame structure). The decisive test was run
       (tools/rtl433/probe_capture.py on g001_868.2M_1000k.cu8): the on-air line
       IS Manchester (confirmed, ~3% illegal pairs), and after decoding, the
       frame is [preamble][EA EA EA EA address][32-byte payload][CRC-16], with the
       hw CRC-16 verifying. So Manchester coding + the 4-byte EAEAEAEA address +
       32-byte payload width are all confirmed from a real capture. (The firmware
       does no *software* Manchester - MC9S08 FUN_ce01/FUN_8ec7 append raw bytes
       to a ring buffer at RAM 0x0253, and the 8051 relays them verbatim - so the
       Manchester is a radio/PHY-layer effect, not a firmware step.)
       RESOLVED (implementation): rtl_433's own FSK_MC_ZEROBIT output was
       reconciled against probe_capture.py's independent decode - both find
       EAEAEAEA and pass CRC-16 on the SAME two captured packets, just at a
       per-packet bit offset that bitbuffer_search already handles. The decoder
       below anchors on the address and CRC-16-gates every frame; both packets
       of g001_868.2M_1000k.cu8 decode with mic=CRC.
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
         _DAT_0140==_DAT_061f) before accepting the reply.
       - BYTE OFFSETS NOW LOCATED (verified against real captures). Reading the
         raw HCS08 assembly of the frame serializer FUN_ce01 gives an exact byte
         formula, tested against g001's two CRC-validated payloads with
         DIFFERENT sub-type bytes - all four independent byte predictions
         matched exactly:
           payload[0] = record[8] + record[0xb] + 12   (a length/sub-type byte;
                        record[0xb] is a per-message-type value, e.g. 1 for the
                        FUN_de1b poll path)
           payload[1] = 0xCC (constant)
           payload[2] = 0x6E (constant, CRC-covered)
           payload[3] = (record[8]&3)<<4 | record[9] | 0x0f   (record[9] was
                        0x40 in one captured packet, 0x80 in the other - a
                        candidate direction/type marker, NOT confirmed)
           payload[4]     = _DAT_013e, LOW byte only (the high byte is never
                             transmitted by this serializer)
           payload[5:7]   = _DAT_0140, full 16-bit big-endian
           payload[7]     = _DAT_0109, LOW byte only (high byte never sent)
           payload[8:10]  = _DAT_010b, full 16-bit big-endian
         This exactly explains the empirically-observed "swapped 3-byte node
         ID" fields between poll and response (docs/rtl433.md "0c"): the
         3-byte group (013e_lo + full 0140) in one packet swaps with (0109_lo +
         full 010b) in the other - i.e. each side sends "my (013e,0140)" and
         echoes back "your (0109,010b)", and the peer mirrors it. Consistent
         with a correlation cookie / pairing handshake, not a physical reading.
         Payload bytes [10:32) (before the CRC-16 trailer) are NOT yet mapped -
         the assembly shows they come from a count-loop with computed/indirect
         addressing that was traced structurally but not fully resolved
         statically (likely more queued sub-messages, since the accounted
         bytes don't fill 32). The decoder below exposes the bytes at [0],[3],
         and the two RAM-word slots by their RAM-address origin (not a
         physical unit - we don't know what they measure yet), alongside the
         full raw payload for continued correlation on the unmapped tail.
       The alarm code table (0x02 no-radio, 0x28 compressor, 0x29-0x2D MV1-5,
       0x2F EEPROM error, ...) and the phase/status S-codes (see docs/eeprom-map.md
       and docs/u2-serial-protocol.md) remain the best decode *targets* for the
       unmapped tail. [U6] is still best resolved by a display-correlated
       capture: change the level / force an alarm and watch which decoded
       payload byte moves.

This decoder anchors on the address, CRC-16-gates every frame, and emits the
CRC-validated 32-byte payload as hex (field semantics [U6] not yet mapped, so
the raw payload is still what captures are correlated against). It is ENABLED:
tested end-to-end in rtl_433 on g001_868.2M_1000k.cu8, both packets of the
exchange decode with mic=CRC, and an unrelated 433 MHz capture produces no
false positives.

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
