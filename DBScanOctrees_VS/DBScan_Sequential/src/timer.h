#ifndef TIMER_H
#define TIMER_H

#include <time.h>
#include <stdint.h>

#define NANOSECS_PER_SEC 1000000000

class Timer {
    int64_t start_time;
public:
    void start() {
        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);
        start_time = start.tv_sec * NANOSECS_PER_SEC + start.tv_nsec;
    }
    double stop() const {
        struct timespec end;
        clock_gettime(CLOCK_MONOTONIC, &end);
        int64_t end_time = end.tv_sec * NANOSECS_PER_SEC + end.tv_nsec;
        return (double) (end_time - start_time) / NANOSECS_PER_SEC;  
    }
};

#endif
