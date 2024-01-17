#include "sp_common/filters/filters.h"
namespace sp_common
{
template <typename T>
T minAbs(T a, T limit)
{
	T sign = (a < 0.0) ? -1.0 : 1.0;
	return sign * fmin(fabs(a),limit);
}

template <typename T>
RampFilter<T>::RampFilter(T acc, T dt) : acc_(acc), dt_(dt), last_value_(0){};

template <typename T>
void RampFilter<T>::input(T input_value)
{
	last_value_ += minAbs<T>(input_value - last_value_, acc_ * dt_);
}

template <typename T>
T RampFilter<T>::output()
{
	return last_value_;
}

template <typename T>
void RampFilter<T>::setAcc(T acc)
{
	if (acc == 0)
		acc_ = 12.0;
	else
		acc_ = acc;
}

template <typename T>
void RampFilter<T>::clear()
{
  last_value_ = 0.;
}

template <typename T>
void RampFilter<T>::clear(T last_value)
{
  last_value_ = last_value;
}


template class RampFilter<double>;
template class RampFilter<float>;
} //namespace sp_common
