#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <deque>
#include <algorithm>



enum class ShaperType { ZV, ZVD, EI, ZVDD };

struct ShapedImpulse {
	std::vector<double> amplitudes; 
	std::vector<double> times;      
};

class InputShaper {
public:
	InputShaper();
	~InputShaper();
	void init(const std::vector<double>& amplitudes, const std::vector<double>& delays,double sampleTime = 0.001);
	ShapedImpulse shaperGenerator(ShaperType type, double freq, double zeta, double V = 0.03);
	double process(double newCommand);

private:
	std::deque<double> buffer_;       
	std::vector<double> amplitudes_;  
	std::vector<int> delaySteps_;     
	double sampleTime_;               
};
