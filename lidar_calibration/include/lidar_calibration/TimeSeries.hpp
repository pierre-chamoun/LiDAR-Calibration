#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

using Timestamp = int64_t;

template <typename T>
struct TimeStamped {
    Timestamp timestamp;
    T data;
};

template <typename T>
class TimeSeries {
    struct ByTimestamp {
        bool operator()(const TimeStamped<T>& a, const TimeStamped<T>& b) const {
            return a.timestamp < b.timestamp;
        }
        bool operator()(const TimeStamped<T>& a, Timestamp t) const { return a.timestamp < t; }
        bool operator()(Timestamp t, const TimeStamped<T>& b) const { return t < b.timestamp; }
    };

    static constexpr std::size_t kCompactThreshold = 100;

   public:
    void insert(Timestamp t, T data) {
        assert(stream_.empty() || t > stream_.back().timestamp);
        stream_.push_back({t, std::move(data)});
    }

    void eraseBefore(Timestamp t) {
        auto it = std::lower_bound(begin(), end(), t, ByTimestamp{});
        offset_ = static_cast<std::size_t>(it - stream_.begin());
        if (offset_ >= kCompactThreshold) {
            stream_.erase(stream_.begin(), stream_.begin() + static_cast<std::ptrdiff_t>(offset_));
            offset_ = 0;
        }
    }

    void popFront() {
        assert(!empty());
        ++offset_;
        if (offset_ >= kCompactThreshold) {
            stream_.erase(stream_.begin(), stream_.begin() + static_cast<std::ptrdiff_t>(offset_));
            offset_ = 0;
        }
    }

    void clear() {
        stream_.clear();
        offset_ = 0;
    }

    std::size_t size() const { return stream_.size() - offset_; }
    bool empty() const { return size() == 0; }

    const TimeStamped<T>& latest() const {
        assert(!empty());
        return stream_.back();
    }

    auto begin() const { return stream_.begin() + static_cast<std::ptrdiff_t>(offset_); }
    auto end() const { return stream_.end(); }

    struct Range {
        using Iterator = typename std::vector<TimeStamped<T>>::const_iterator;
        Iterator first, last;
        Iterator begin() const { return first; }
        Iterator end() const { return last; }
    };

    Range range(Timestamp t0, Timestamp t1) const {
        return {std::lower_bound(begin(), end(), t0, ByTimestamp{}),
                std::upper_bound(begin(), end(), t1, ByTimestamp{})};
    }

    const TimeStamped<T>* at(Timestamp t) const {
        auto it = std::lower_bound(begin(), end(), t, ByTimestamp{});
        if (it == end() || it->timestamp != t) return nullptr;
        return &*it;
    }

    const TimeStamped<T>* closestTo(Timestamp t) const {
        auto hi = std::lower_bound(begin(), end(), t, ByTimestamp{});
        if (hi == end()) return hi == begin() ? nullptr : &*std::prev(hi);
        if (hi == begin()) return &*hi;
        const auto& lo = *std::prev(hi);
        return (t - lo.timestamp) <= (hi->timestamp - t) ? &lo : &*hi;
    }

    const TimeStamped<T>* floor(Timestamp t) const {
        auto it = std::upper_bound(begin(), end(), t, ByTimestamp{});
        if (it == begin()) return nullptr;
        return &*std::prev(it);
    }

    const TimeStamped<T>* ceil(Timestamp t) const {
        auto it = std::lower_bound(begin(), end(), t, ByTimestamp{});
        if (it == end()) return nullptr;
        return &*it;
    }

   private:
    std::vector<TimeStamped<T>> stream_;
    std::size_t offset_ = 0;
};
