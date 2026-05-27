#pragma once
#include <stdint.h>
#include <stdbool.h>

// Lock-free Single-Producer Single-Consumer (SPSC) ring buffer.
//
// Contract:
//   - Only ONE producer may call push() — Core 1 in this firmware.
//   - Only ONE consumer may call pop()  — Core 0 in this firmware.
//   - N must be a power of 2.
//
// Index scheme: head and tail are raw (unwrapped) monotonically-increasing
// counters. Wrap-around into buf[] is done by masking with (N-1).
// This makes the full/empty checks simple and avoids double-masking bugs.
//
// Overflow policy: if push() is called when the buffer is full it overwrites
// the oldest entry and returns false so the caller can log a data-loss warning.

template <typename T, uint32_t N>
class RingBuffer {
    static_assert((N & (N - 1)) == 0, "RingBuffer: N must be a power of 2");

    T                buf[N];
    volatile uint32_t head = 0;  // written by producer (Core 1); read by consumer
    volatile uint32_t tail = 0;  // written by consumer (Core 0); read by producer

public:
    // Push from producer (Core 1).
    // If the buffer is full, the oldest entry is overwritten.
    // Returns false when an overwrite (data loss) occurred.
    bool push(const T &item) {
        uint32_t h   = head;
        bool     full = ((h - tail) == N);

        buf[h & (N - 1)] = item;

        // Memory barrier: ensure the write to buf[] is visible to the consumer
        // before we advance head.
        __sync_synchronize();

        head = h + 1;               // unwrapped increment

        if (full) {
            // Drop the oldest entry so the consumer never sees a half-written slot.
            tail = tail + 1;
        }

        return !full;
    }

    // Pop from consumer (Core 0).
    // Returns false if the buffer is empty; out is not modified.
    bool pop(T &out) {
        uint32_t t = tail;
        if (t == head) return false;

        out = buf[t & (N - 1)];

        // Memory barrier: ensure the read of buf[] completes before we advance
        // tail so the producer doesn't recycle the slot too early.
        __sync_synchronize();

        tail = t + 1;               // unwrapped increment

        return true;
    }

    // Number of items currently in the buffer.
    // Valid range: [0, N]. Safe to call from either core (approximate snapshot).
    uint32_t size(void) const {
        return head - tail;
    }

    // Convenience helpers.
    bool empty(void) const { return head == tail; }
    bool full(void)  const { return (head - tail) == N; }
};
