#pragma once
#include <cstdint>

namespace SGE {

// An Entity is a 32-bit handle: 24-bit index + 8-bit generation.
//
// The index is a slot number; the generation is bumped each time that slot is
// recycled. Storing the generation in the handle lets us detect a "stale"
// handle (one that refers to a slot whose entity was destroyed and reused) —
// the index will still be valid but the generation won't match.
struct Entity {
    uint32_t Value = Invalid;

    static constexpr uint32_t IndexBits = 24;
    static constexpr uint32_t IndexMask = (1u << IndexBits) - 1; // 0x00FFFFFF
    static constexpr uint32_t Invalid   = 0xFFFFFFFFu;

    constexpr Entity() = default;
    constexpr explicit Entity(uint32_t v) : Value(v) {}

    static constexpr Entity Make(uint32_t index, uint32_t generation) {
        return Entity{ (generation << IndexBits) | (index & IndexMask) };
    }

    constexpr uint32_t Index()      const { return Value & IndexMask; }
    constexpr uint32_t Generation() const { return Value >> IndexBits; }
    constexpr bool     IsValid()    const { return Value != Invalid; }

    constexpr bool operator==(const Entity& o) const { return Value == o.Value; }
    constexpr bool operator!=(const Entity& o) const { return Value != o.Value; }
};

} // namespace SGE
