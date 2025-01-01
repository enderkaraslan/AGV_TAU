#pragma once

#include <chrono>

class Timer {
public:
    Timer();
    void start();
    void stop();
    long long elapsedMilliseconds() const;
    bool isExpired(long long durationMilliseconds) const;

private:
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point stop_time_;
    bool is_running_;
};