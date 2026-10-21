#pragma once

#include <cmath>
#include <cstdint>

namespace sim {

/// splitmix64：无状态哈希，用来把 (seed, sample_index, channel) 变成一条独立随机流。
inline std::uint64_t splitmix64(std::uint64_t x) noexcept {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

inline std::uint64_t mixKey(std::uint64_t seed, std::uint64_t index, std::uint64_t channel) noexcept {
    return splitmix64(seed ^ splitmix64(index * 0x100000001B3ull + channel));
}

/// 由 key 派生的确定性随机流。
/// 关键点：没有「全局状态」——同一 key 无论何时、以何种顺序被求值，结果都一致。
class Rng {
public:
    explicit Rng(std::uint64_t seed) noexcept : s_(seed) {}

    [[nodiscard]] std::uint64_t next() noexcept {
        s_ += 0x9E3779B97F4A7C15ull;
        return splitmix64(s_);
    }

    /// [0, 1)
    [[nodiscard]] double uniform() noexcept {
        return static_cast<double>(next() >> 11) * 0x1.0p-53;
    }

    /// 标准正态（Box–Muller）
    [[nodiscard]] double gauss() noexcept {
        const double u1 = 1.0 - uniform();  // (0, 1]，避开 log(0)
        const double u2 = uniform();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586476925 * u2);
    }

private:
    std::uint64_t s_;
};

}  // namespace sim
