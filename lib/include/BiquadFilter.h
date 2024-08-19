#pragma once
#include <iostream>
#include <vector>

class BiquadFilter
{
private:
    double a0, a1, a2, b0, b1, b2;
    double x1, x2, y1, y2;

public:
    BiquadFilter(double a0, double a1, double a2,double b0, double b1, double b2)
        : a0(a0), a1(a1), a2(a2), b0(b0), b1(b1), b2(b2), x1(0), x2(0), y1(0), y2(0) {}
    
    ~BiquadFilter(){}
    double process(double input);
};