#ifndef RUNING_AVERAGE_H_
#define RUNING_AVERAGE_H_

template <int SIZE> class RunningAverage {
    float buffer[SIZE] = {0};
    int next_running_average = 0;

public:
    float filter(float value) {
        buffer[next_running_average++] = value;
        if (next_running_average >= SIZE) {
            next_running_average = 0;
        }
        float res = 0;
        for (int i = 0; i < SIZE; ++i) {
            res += buffer[i];
        }
        res /= SIZE;
        return res;
    }
};

#endif