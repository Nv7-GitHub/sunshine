#pragma once
#include <stdint.h>
#include <atomic>

// Lock-free SPSC ring buffer.
// Producer (Core 1) calls push(); consumer (Core 0) calls pop().
// On overflow, oldest entry is silently overwritten (no tail mutation by producer).
// N must be a power of 2.
template <typename T, uint32_t N>
class RingBuffer {
    static_assert((N & (N - 1)) == 0, "N must be a power of 2");
    static_assert(std::is_trivially_copyable<T>::value, "RingBuffer<T>: T must be trivially copyable");

    T buf[N];
    std::atomic<uint32_t> head{0};  // written only by producer
    std::atomic<uint32_t> tail{0};  // written only by consumer

public:
    // Push from producer (Core 1).
    // If full, drops the item (returns false) to avoid write/read race.
    // Returns true if the item was enqueued, false if dropped due to full buffer.
    bool push(const T &item) {
        uint32_t h = head.load(std::memory_order_relaxed);
        uint32_t t = tail.load(std::memory_order_acquire);
        if ((h - t) == N) return false;   // full — drop, never overwrite

        buf[h & (N - 1)] = item;
        head.store(h + 1, std::memory_order_release);
        return true;
    }

    // Pop from consumer (Core 0). Returns false if empty.
    bool pop(T &out) {
        uint32_t t = tail.load(std::memory_order_relaxed);
        uint32_t h = head.load(std::memory_order_acquire);
        if (t == h) return false;

        out = buf[t & (N - 1)];
        tail.store(t + 1, std::memory_order_release);
        return true;
    }

    uint32_t size() const {
        uint32_t h = head.load(std::memory_order_acquire);
        uint32_t t = tail.load(std::memory_order_acquire);
        return h - t;  // unwrapped, always in [0, N]
    }

    bool empty() const { return size() == 0; }
    bool full()  const { return size() == N; }
};
