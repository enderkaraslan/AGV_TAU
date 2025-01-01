#include "Timer.h"

Timer::Timer() : is_running_(false) {}

void Timer::start() {
    start_time_ = std::chrono::steady_clock::now();
    is_running_ = true;
}

void Timer::stop() {
    stop_time_ = std::chrono::steady_clock::now();
    is_running_ = false;
}

long long Timer::elapsedMilliseconds() const {
    if (is_running_) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_).count();
    } else {
        return std::chrono::duration_cast<std::chrono::milliseconds>(stop_time_ - start_time_).count();
    }
}

bool Timer::isExpired(long long durationMilliseconds) const {
    return elapsedMilliseconds() >= durationMilliseconds;
}