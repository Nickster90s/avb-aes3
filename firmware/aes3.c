// AES3 ↔ AVTP Audio Bridge
// Moves samples between AES3 hardware FIFOs (CSR) and AVTP ring buffers.

#include "aes3.h"

#include <generated/csr.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// AES3 TX Channel Status (IEC 60958-3 professional)
// ---------------------------------------------------------------------------

// Byte 0: professional=1, not_audio=0, emphasis=none, lock=unlocked, fs_mode=not_indicated
// Byte 1: channel_mode=2ch, user_bits=default
// Byte 2: source=unspecified, channel_number=unspecified
// Byte 3: fs=48kHz(0100), clock_accuracy=level_II
// Byte 4: sample_length=24bit, original_fs=48kHz
static void aes3_set_default_channel_status(void)
{
    // Professional format, 2-channel, 48 kHz, 24-bit
    // Byte 0: bit0=1 (professional)
    // Byte 3: bits[3:0]=0100 (48 kHz), bits[5:4]=00 (Level II clock accuracy)
    // Byte 4: bits[2:0]=010 (24-bit max word length), bit3=1 (word length = max)
    uint32_t cs0 = 0x01;       // Byte 0: professional
    cs0 |= (0x00 << 8);       // Byte 1: 2-channel mode
    cs0 |= (0x00 << 16);      // Byte 2: unspecified source/channel
    cs0 |= (0x04 << 24);      // Byte 3: 48 kHz (0100 in bits 3:0)

    uint32_t cs1 = 0x0A;      // Byte 4: 24-bit (010) + word_len=max (1) = 0x0A

    aes3_tx_cs0_write(cs0);
    aes3_tx_cs1_write(cs1);
    aes3_tx_cs2_write(0);
    aes3_tx_cs3_write(0);
    aes3_tx_cs4_write(0);
    aes3_tx_cs5_write(0);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void aes3_init(aes3_state_t *s)
{
    s->rx_locked        = 0;
    s->rx_was_locked    = 0;
    s->rx_sample_count  = 0;
    s->rx_overrun_count = 0;
    s->tx_sample_count  = 0;
    s->tx_underrun_count = 0;

    aes3_set_default_channel_status();

    printf("[AES3] Initialized. RX locked=%lu\n", (unsigned long)aes3_rx_locked_read());
}

// ---------------------------------------------------------------------------
// Poll
// ---------------------------------------------------------------------------

void aes3_poll(aes3_state_t *s, audio_ring_t *rx_ring, audio_ring_t *tx_ring)
{
    // --- Check lock status ---
    s->rx_locked = aes3_rx_locked_read();
    if (s->rx_locked && !s->rx_was_locked) {
        printf("[AES3] RX locked (96k=%lu)\n", (unsigned long)aes3_rx_is_96k_read());
    } else if (!s->rx_locked && s->rx_was_locked) {
        printf("[AES3] RX lock lost\n");
    }
    s->rx_was_locked = s->rx_locked;

    // --- AES3 RX → AVTP TX ring ---
    // Drain AES3 RX FIFO into the AVTP talker's ring buffer.
    if (rx_ring && s->rx_locked) {
        uint32_t rx_level = aes3_rx_level_read();
        while (rx_level > 0) {
            if (audio_ring_space(rx_ring) == 0) {
                s->rx_overrun_count++;
                break;
            }
            // Reading _rx_audio_l pops the FIFO and latches both L and R.
            int32_t left  = (int32_t)aes3_rx_audio_l_read();
            int32_t right = (int32_t)aes3_rx_audio_r_read();

            // Sign-extend from 24-bit to 32-bit
            if (left  & 0x800000) left  |= (int32_t)0xFF000000;
            if (right & 0x800000) right |= (int32_t)0xFF000000;

            audio_ring_write(rx_ring, left, right);
            s->rx_sample_count++;
            rx_level--;
        }
    }

    // --- AVTP RX ring → AES3 TX ---
    // Feed the AES3 TX FIFO from the AVTP listener's ring buffer.
    if (tx_ring) {
        uint32_t tx_space = 64 - aes3_tx_level_read();
        while (tx_space > 0 && audio_ring_count(tx_ring) > 0) {
            int32_t left, right;
            audio_ring_read(tx_ring, &left, &right);

            // Write 24-bit samples (mask to 24 bits)
            aes3_tx_audio_l_write(left & 0x00FFFFFF);
            aes3_tx_audio_r_write(right & 0x00FFFFFF);
            aes3_tx_push_write(1);

            s->tx_sample_count++;
            tx_space--;
        }
    }
}
