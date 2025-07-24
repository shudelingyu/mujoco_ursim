#include "inputshaper.hpp"
#include "modern_robotics.hpp"


InputShaper::InputShaper() {

};
InputShaper::~InputShaper() {

};


void InputShaper::init(const std::vector<double>& amplitudes, 
	const std::vector<double>& delays,    
	double sampleTime) {

	amplitudes_ = amplitudes;
	sampleTime_ = sampleTime;
	std::cout << "amplitudes_ ";
	for (auto& i : amplitudes_)
		std::cout << i << " ";
	std::cout << std::endl;

	std::cout << "delays ";
	for (auto& i : delays)
		std::cout << i << " ";
	std::cout << std::endl;

	for (double t : delays) {
		delaySteps_.push_back(static_cast<int>(std::round(t / sampleTime)));
	}
	buffer_.resize(*std::max_element(delaySteps_.begin(), delaySteps_.end()) + 1, 0.0);
}

ShapedImpulse InputShaper::shaperGenerator(ShaperType type, double freq, double zeta,double V) {
	if (freq <= 0 || zeta < 0 || zeta >= 1)
		throw std::invalid_argument("freq or zeta error!!!");

	const double omega_d = 2 * M_PI * freq * std::sqrt(1 - zeta * zeta); 
	const double T_d = 1 / freq; 
	const double K = std::exp(-zeta * M_PI / std::sqrt(1 - zeta * zeta)); 

	ShapedImpulse result;

	switch (type) {
	case ShaperType::ZV: 
		result.amplitudes = { 1 / (1 + K), K / (1 + K) };
		result.times = { 0, 0.5 * T_d };
		break;

	case ShaperType::ZVD:
		result.amplitudes = { 1 / (1 + 2 * K + K * K), 2 * K / (1 + 2 * K + K * K), K * K / (1 + 2 * K + K * K) };
		result.times = { 0, 0.5 * T_d, T_d };
		break;

	case ShaperType::EI: 
		result.amplitudes = { (1 + V) / 4, (1 - V) / 2, (1 + V) / 4 };
		result.times = { 0, M_PI / freq, 2 * M_PI / freq };
		break;

	case ShaperType::ZVDD:
		result.amplitudes = { 1, 3 * K, 3 * K * K, K * K * K };
		double denom = 1 + 3 * K + 3 * K * K + K * K * K;
		for (auto& amp : result.amplitudes) amp /= denom;
		result.times = { 0, 0.5 * T_d, T_d, 1.5 * T_d };
		break;

	}
	return result;
}

double InputShaper::process(double newCommand) {
	buffer_.push_front(newCommand);
	if (buffer_.size() > delaySteps_.back() + 1) {
		buffer_.pop_back();
	}

	double shapedValue = 0.0;
	for (size_t i = 0; i < amplitudes_.size(); ++i) {
		int idx = delaySteps_[i];
		if (idx < static_cast<int>(buffer_.size())) {
			shapedValue += amplitudes_[i] * buffer_[buffer_.size() - 1 - idx];
		}
	}
	std::cout << "newCommand-shapedValue " << newCommand - shapedValue << std::endl;
	return shapedValue;
}

