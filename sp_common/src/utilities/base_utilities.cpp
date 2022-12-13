#include "sp_common/utilities/base_utilities.h"

namespace sp_common
{
	template <typename T>
	T getParam(ros::NodeHandle &pnh, const std::string &param_name, const T &default_val)
	{
		T param_val;
		pnh.param<T>(param_name, param_val, default_val);
		return param_val;
	}
} // namespace sp_common
