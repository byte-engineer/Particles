#pragma once
#include <chrono>
#include <iostream>


namespace utils
{
    // class Timer
    // {
    // private:
    //     std::chrono::time_point<std::chrono::steady_clock> start;
    //     std::chrono::time_point<std::chrono::steady_clock> end;

    // public:
    //     Timer()
    //     {
    //         start = std::chrono::high_resolution_clock::now();
    //     }

    //     ~Timer()
    //     {
    //         end = std::chrono::high_resolution_clock::now();

    //         auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    //         std::cout << "Time: "<< "( " << duration.count()*0.001 << "ms )" << std::endl;
    //     }
    // };


    float random(float min, float max)
    {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rand(min, max);
    return rand(gen);
    }

    float map(float value, float min, float max, float outMin, float outMax)
    {
        return (value - min) * (outMax - outMin) / (max - min) + outMin;
    }

    float clamp(float value, float min, float max)
    {
        if (value > max)
            return max;
        else if (value < min)
            return min;
        return value;
    }



    Color tempColor(float value)
    {

        int valueInt = (int)map(clamp(value, 0, 100), 0, 100, 0, 255);
        unsigned int _value = 255 - valueInt;
        unsigned short value_ = valueInt - 255;      // 0 -> 255

        return Color { (unsigned char)value_, 128, (unsigned char)_value, 255};
    }

    Color randColor(float min=0, float max=255 )
    {
        return Color { (unsigned char)random(min, max), (unsigned char)random(min, max), (unsigned char)random(min, max), 255};
    }

} // utils