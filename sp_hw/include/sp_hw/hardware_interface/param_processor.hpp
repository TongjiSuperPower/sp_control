#include <ros/ros.h>
//#define PARAM_DEBUG

/*
 * @brief Used when &value is a TypeDouble or TypeInt
 */
inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue &value)
{
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        // operater()
        const int temp = value;
        return (double)temp;
    }
    else
        return value;
}

/*
 * @brief Used when &value is a TypeStruct
 */
inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue &value, const std::string &field)
{
    if (value.hasMember(field))
    {
        ROS_ASSERT((value[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
                   (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
#ifdef PARAM_DEBUG
        std::cout << "DEBUG : " << field << " " << value[field] << std::endl;
#endif
        return xmlRpcGetDouble(value[field]);
    }
    else
    {
        ROS_WARN_STREAM("Error :" << field << " xmlRpcGetDouble FAILED!");
        return (double)0.0;
    }
}