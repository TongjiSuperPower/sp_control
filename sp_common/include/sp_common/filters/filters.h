#pragma once

#include <cmath>
namespace sp_common
{
template <typename T>
class RampFilter
{
public:
	RampFilter(T acc, T dt);
	void input(T input_value);
	T output();
	void setAcc(T acc);
	void clear();
	void clear(T last_value);
private:
    T acc_;
	T dt_;
	T last_value_;
};

} //namespace sp_common
