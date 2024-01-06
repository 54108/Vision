#include "fps.hpp"
fps::FPS::FPS()
{
    cnt = 0;
    time1 = 0.0;
    time2 = 0.0;
    time = 0.0;
    fps = 0.0;
    max = 0.0;
    min = 99999999.0;
    average = 0.0;
    total = 0.0;

    name_ = "Global";
}

fps::FPS::FPS(std::string _name)
{
    cnt = 0;
    time1 = 0.0;
    time2 = 0.0;
    time = 0.0;
    fps = 0.0;
    max = 0.0;
    min = 99999999.0;
    average = 0.0;
    total = 0.0;

    name_ = _name;
}

void fps::FPS::calculateFPS()
{
    time2 = cv::getTickCount();
    time = (time2 - time1) / cv::getTickFrequency();
    fps = 1.f / time;

    if (++cnt > 10)
    {
        if (fps > max)
        {
            max = fps;
        }
        if (fps < min)
        {
            min = fps;
        }

        total += time;
        average = total / (cnt - 10);
    }
    displayFPS();
}

void fps::FPS::calculateFPSGlobal()
{
    time2 = cv::getTickCount();
    time = (time2 - time1) / cv::getTickFrequency();
    fps = 1.f / time;

    if (++cnt > 10)
    {
        if (fps > max)
        {
            max = fps;
        }
        if (fps < min)
        {
            min = fps;
        }
        total += time;
        average = total / ((cnt)-10);
    }

    last_time = average;
}

void fps::FPS::getTick()
{
    time1 = cv::getTickCount();
}

double fps::FPS::lastTime()
{
    return last_time;
}
