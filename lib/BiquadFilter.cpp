#include "BiquadFilter.h"

double BiquadFilter::process(double input)
{
    if(a0 == 0) {
        // Handle error, a0 should not be 0.
        return 0;
    }

    double output = (b0/a0) * input + (b1/a0) * x1 + (b2/a0) * x2 - (a1/a0) * y1 - (a2/a0) * y2;
    
    // 更新延迟线
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;

    return output;
}