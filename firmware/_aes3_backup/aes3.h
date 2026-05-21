// AES3 ↔ AVTP Audio Bridge
// Moves samples between AES3 hardware FIFOs (CSR) and AVTP ring buffers.

#ifndef AES3_H
#define AES3_H

#include <stdint.h>
#include "avtp.h"

typedef struct {
    uint8_t  rx_locked;
    uint8_t  rx_was_locked;       // For edge detection (lock gained/lost)
    uint32_t rx_sample_count;
    uint32_t rx_overrun_count;    // Ring buffer full, dropped samples
    uint32_t tx_sample_count;
    uint32_t tx_underrun_count;   // FIFO empty, couldn't feed AES3 TX
} aes3_state_t;

// Initialize AES3 bridge.  Sets default TX channel status (pro audio, 48 kHz).
void aes3_init(aes3_state_t *s);

// Poll: transfer samples between AES3 FIFOs and AVTP ring buffers.
// rx_ring: AES3 RX → this ring (feeds AVTP talker)
// tx_ring: this ring → AES3 TX (fed by AVTP listener)
void aes3_poll(aes3_state_t *s, audio_ring_t *rx_ring, audio_ring_t *tx_ring);

#endif // AES3_H
