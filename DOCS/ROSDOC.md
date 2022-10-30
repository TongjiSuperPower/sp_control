



# 说明

基于ros1开发，所以用的是moveit1、rviz等ros1的包。

若要迁移至ros2，应至少对代码内有关通讯部分的代码进行更改（不包括socketCAN）

队内使用roscpp进行开发，因此文档也只包括cpp的实现，视情况会增加部分命令行的使用。

易混淆的概念和名词会使用英文书写以方便理解。

感谢廖恰源的rm-control项目。



# 基础概念

## 命名空间 Name & Namespace

由于ROS使用图资源（Graph Resource）进行资源的管理，Nodes、Topics、Parameters、Services都基于此创建和运行，因此理解命名空间就显得很重要。

在ROS中总共有四种[命名方式](http://wiki.ros.org/Names)，分别为base、relative、global、private。

- relative：默认情况下，解析是**相对于节点的命名空间**完成的。例如，节点 `/engineer/node1` 具有命名空间 `/engineer`，因此名称`node2` 将解析为 `/engineer/node2`。
- base：没有任何命名空间限定符的名称。base实际上是relative的子类，并具有相同的解析规则。基本名称最常用于初始化节点名称。
- global：以“/”开头的名称是- 它们被认为是完全解析的。应尽可能避免全局名称，因为它们会限制代码可移植性。
- private：以“~”开头的名称。它们将节点的名称转换为命名空间。例如，命名空间 `/engineer/` 中的`Node1` 具有私有命名空间 `/engineer/Node1`。私有名称对于通过参数服务器将参数传递到特定节点非常有用。



给出一些ROS Wiki上的例子：

| Node      | Relative                | Global                | Private                       |
| --------- | ----------------------- | --------------------- | ----------------------------- |
| /node1    | bar---->/bar            | /bar---->/bar         | ~bar---->/node1/bar           |
| /wg/node2 | bar---->/wg/bar         | /bar---->/bar         | ~bar---->/wg/node2/bar        |
| /wg/node3 | foo/bar---->/wg/foo/bar | /foo/bar---->/foo/bar | ~foo/bar---->wg/node3/foo/bar |

上述四种方式，relative和private之间的区别是容易混淆的。我们可以用文件夹的层级结构去理解其不同。假设我们有一个两级文件夹：

```
|---wg
    |---node1
```

我们在node1下使用relative方式创建bar，则形如：

```
|---wg
    |---node1
    |---bar
```

而同样在node1下使用private方式创建bar，则形如：

```
|---wg
    |---node1
        |---bar
```



在实际开发中，NodeHandle的命名空间是我们格外关注的，通常我们会使用`relative`和`private`的命名方式。下面给出若干例子：

```c++
/*       Relative        */
ros::NodeHandle nh("my_namespace");   	// nh  in <node_namespace>/my_namespace
ros::NodeHandle nh1("ns1");				// nh1 in <node_namespace>/ns1
ros::NodeHandle nh2(nh1, "ns2");		// nh2 in <node_namespace>/ns1/ns2
/*       Private         */
ros::NodeHandle nh3("~my_namespace");	// nh3 in <node_namespace>/<node_name>/my_namespace
```

应当注意`<node_namespace>`和`<node_name>`是完全两个概念，前者指`node`所在的命名空间，后者指的是`node`的名字，勿混淆。



# 参数服务器 Parameter Server

ROS中使用参数服务器存储用户加载的参数。他支持多种数据类型，例如：整型、字符串、浮点数、布尔、字典、数组。

roscpp的参数服务器类API支持上述的所有数据类型，但不同数据类型API的使用难度并不一致，显然基础类型的数据类型（**整形、布尔、字符串、浮点**）使用起来会较为方便，而诸如**字典、数组**这样的组合类型则还需要借助**`XmlRpc::XmlRpcValue Class`**（before ROS Fuerte）或者`std::map`进行实现。

目前`std::map`支持的参数类型只有`bool、float、double、int、string`，因此如果要使用多级字典（比如<string,map>）则仍只能使用`XmlRpcValue`类型进行解析和检索。



## 参数上载 Param Upload

参数上载可分为两步：编制 和 上载。

编制：我们经常使用一种有层次的结构声明数据，也即是YAML格式语言。参看下述以YAML格式书写的电机参数文件（config）。该文件在工程中用于定义电机的各项参数。可以看到在actuator_coefficient中我们定义了RM比赛中常用的两个电机的参数。

>  act2effort参数是使用电流转矩系数近似得出的，其真实情况应是非线性的，但以平衡步兵的控制情况来看，该非线性误差可以不考虑，故不深究。

```yaml
actuator_coefficient:
  rm_3508: # RoboMaster 3508 without reducer
    act2pos: 0.0007669903  # 2PI/8192
    act2vel: 0.1047197551   # 2PI/60
    act2effort: 1.90702994e-5  # 20/16384*0.0156223893
    effort2act: 52437.561519   # 1/act2effort
    max_out: 16384
  rm_2006: # RoboMaster 2006 motor
    act2pos: 2.13078897e-5  # 2PI/8192*(1/36)
    act2vel: 0.0029088820   # 2PI/60*(1/36)
    act2effort: 0.00018  #10/10000*0.18
    effort2act: 5555.5555555   # 1/act2effort
    max_out: 10000
```

上传：在launch文件中使用`<rosparam>`标签配合`load`指令完成上传。下例展示了`chassis_controller`的参数上载写法。

> `<rosparam>` 标记也可以放在`node`标记内，在这种情况下，参数命名空间使用`Private Name`解析。

```yaml
<launch>
	<rosparam file="$(find chassis_controller)/config/engineer.yaml" command="load"/>
</launch>
```



## 参数下载 Param Download

参数下载可分为两步：下载 和 赋值。

下载：我们使用ros::NodeHandle.getParam()获取yaml文件中的参数

```cpp
XmlRpc::XmlRpcValue xml_rpc_value;
controller_nh.getParam("yamlName", xml_rpc_value);
```

赋值：根据yaml中具体的数据结构将值赋到cpp文件中的unordered_map结构中。

同样以这个yaml文件为例

```yaml
actuator_coefficient:
  rm_3508: # RoboMaster 3508 without reducer
    act2pos: 0.0007669903  # 2PI/8192
    act2vel: 0.1047197551   # 2PI/60
    act2effort: 1.90702994e-5  # 20/16384*0.0156223893
    effort2act: 52437.561519   # 1/act2effort
    max_out: 16384
  rm_2006: # RoboMaster 2006 motor
    act2pos: 2.13078897e-5  # 2PI/8192*(1/36)
    act2vel: 0.0029088820   # 2PI/60*(1/36)
    act2effort: 0.00018  #10/10000*0.18
    effort2act: 5555.5555555   # 1/act2effort
    max_out: 10000
```

设cpp文件中定义了这样的unordered_map

```cpp
struct Actcoeff
{
   double act2pos，act2vel, act2effort, effort2act;
   int max_out;
}
std::unordered_map<std::string, Actcoeff>* MotorPtr;
```

如下赋值

```cpp
ROS_ASSERT(xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeStruct);
try
{
    for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
    {
        if (!it->second.hasMember("act2pos"))//it先指向名为rm_3508的map，it->second表示rm_3508的value，即下一级                                                //map。
                                             //使用.hasMember方法查找下一级map中是否存在名为act2pos的key
        { 
            ROS_ERROR_STREAM("Motor " << it->first << " has no associated act2pos.");
            continue;
        }
        else if (!it->second.hasMember("act2vel"))
        {
            ROS_ERROR_STREAM("Motor " << it->first << " has no associated act2vel.");
            continue;
        }
        else if (!it->second.hasMember("act2effort"))
        {
            ROS_ERROR_STREAM("Motor " << it->first << " has no associated act2effor.");                                                                                
            continue;
        }
        else if (!it->second.hasMember("effort2act"))
        {
            ROS_ERROR_STREAM("Motor " << it->first << " has no associated effort2act.");
            continue;
        }
        else if (!it->second.hasMember("max_out"))
        {
            ROS_ERROR_STREAM("Motor " << it->first << " has no associated max_out.");
            continue;
        }
        MotorPtr[it->first]["act2pos"];
        MotorPtr[it->first]["act2vel"];
        MotorPtr[it->first]["act2effort"];
        MotorPtr[it->first]["effort2act"];
        MotorPtr[it->first]["max_out"];    
    }
}  
```

尝试输出

```cpp
std::cout << MotorPtr["rm2006"]["act2pos"] << endl;

0.0007669903
```

理论上多层`XmlRpc::XmlRpcValue`实际上就是多层`std::map`

`XmlRpc::XmlRpcValue`具有多种`type`，其中` TypeStruct`对应的`value`为

```cpp
 typedef std::map<std::string, XmlRpcValue> ValueStruct;
```

即`TypeStruct`类型的`XmlRpc::XmlRpcValue`可视为一个`std::map`，使用`std::string`指向其他类型的`XmlRpc::XmlRpcValue`

故`XmlRpc::XmlRpcValue`类型的数据可以使用`std::map`定义的函数与迭代器。

