#include "time_measurement.h"

#include<fstream>
#include<iostream>

using namespace std;

TimeMeasurement::TimeMeasurement()
{
	start_ticks = 0;
	end_ticks = 0;
}

void TimeMeasurement::start()
{
	start_ticks = SDL_GetTicks();
}

void TimeMeasurement::end()
{
	end_ticks = SDL_GetTicks();

    if (end_ticks > start_ticks)
    diff_ticks = end_ticks-start_ticks;
    int sec = (diff_ticks)/1000;
    int ms = (diff_ticks-sec*1000);

    cout << "   - Consumed time: " << sec << " s, " << ms << " ms\n";
}
