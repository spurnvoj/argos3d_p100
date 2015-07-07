#ifndef __TIME_MEASUREMENT_H__
#define __TIME_MEASUREMENT_H__

#include <vector>
#include <SDL/SDL.h>

class TimeMeasurement
{
private:
    Uint32 start_ticks;
    Uint32 end_ticks;
    Uint32 diff_ticks;
public:
    TimeMeasurement();

    void start();
    void end();
};
#endif
