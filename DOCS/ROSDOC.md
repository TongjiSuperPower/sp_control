



# 说明

1. 感谢廖恰源的rm-control项目。

2. 基于ros1开发，所以用的是moveit1、rviz等ros1的包。若要迁移至ros2，应至少对代码内有关通讯部分的代码进行更改（不包括socketCAN）

3. 使用roscpp进行开发，因此文档也只包括cpp的实现，视情况会增加部分命令行的使用。

4. 易混淆的概念和名词会使用英文书写以方便区分和理解。



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



# 参数服务器  Parameter Server

ROS中使用参数服务器存储用户加载的参数。他支持多种数据类型，例如：整型、字符串、浮点数、布尔、字典、数组。

roscpp的参数服务器类API支持上述的所有数据类型，但不同数据类型API的使用难度并不一致，显然基础类型的数据类型（**整形、布尔、字符串、浮点**）使用起来会较为方便，而诸如**字典、数组**这样的组合类型则还需要借助**`XmlRpc::XmlRpcValue Class`**（before ROS Fuerte）或者`std::map`进行实现。

目前`std::map`支持的参数类型只有`bool、float、double、int、string`，因此如果要使用多级字典（比如<string,map>）则仍只能使用`XmlRpcValue`类型进行解析和检索。



## 参数上载  Param Upload

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
   double act2pos，act2vel, act2effort, effort2act, max_out;
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

> 虽然同样能使用迭代器，但同时应当注意有些许函数是不太一样的，比如在XmlRpcValue中定义了hasMember的函数，而显然unorodered_map中没有这种用法。

# 项目结构

## sp_control

控制层，使用`controller`来实现底盘、云台、机械臂的控制

TODO

```cpp
sp_control
|---chassis_controller//底盘控制器
    |---config
	|---include
    |---launch
    |---src
    	|---chassis_base.cpp//底盘基类
    	|---roller_wheel.cpp//麦轮底盘类
|---engineer_gazebo//gazebo
|---gimbal_controller//云台控制器
	|---config
	|---include
    |---launch
    |---src
    	|---gimbal_controller.cpp//云台控制器类
|---manipulator_controller//机械臂控制器
|---manipulator_moveit_config//机械臂moveit配置
|---sentry_communicator//哨兵通信

```

## sp_description

模型文件夹，存放工程机器人模型

```cpp
sp_description
|---launch
|---urdf
    |---common
    	|---lidar2d.urdf.xacro
    	|---mecanum_wheel.urdf.xacro//通用麦轮模型
    |---engineer
    	|---engineer_chassiss//工程底盘总装
    	|---engineer_manipulator//工程机械臂总装
    	|---fpv_module//工程云台总装
    |---engineer.xacro//工程总装
    |---kinect.xacro//kinect相机
    |---manipulator_transmission.xacro//机械臂transmission
    
```



## sp_hw

抽象硬件层，提供`read`和`write`两个方法通过`can`总线实现与机器人的交互。

TODO

```cpp
sp_hw
|---config
    |---actuator_coefficient.yaml//电机参数
    |---hw_config.yaml//can总线参数
|---include
    |---hardware_interface
    	|---can_bus.hp
    	|---data_types.hpp//定义传输数据类型结构
    	|---hardware_interface.hpp
    	|---param_processor.hpp
    	|---socketcan.h
    |---hardware_interface_loader.hpp
|---launch
    |---sp_hw.launch//加载参数
|---src
    |---hardware_interface
    	|---can_bus.cpp//can总线,上接hardware_interface.cpp,读写自定义can协议帧
    	|---haraware_interface.cpp//硬件面板，提供读写函数
    	|---parse.cpp//加载执行系数和执行数据
    	|---socketcan.cpp//通信底层，上接can_bus,将can帧装入buffer发出或读取buffer中的can帧
    |---hardware_interface_loader.cpp//循环进程，不断更新抽象硬件层的数据
    |---sp_hw.cpp//主进程，开启硬件抽象层并开启控制循环
```



# 实时工具  realtime_tools

## 线程安全的数据获取  RealtimeBuffer

在控制器设计中，我们常会创建 `Subscriber` 用于接收 `topic` 发出的控制指令。控制器为我们的主线程，由 `controller manager` 管理，可认为是实时线程（RT）。而 `Subscriber` 可简单理解为一个非实时线程（NonRT），那么在实际运行中**有可能出现同时接收指令和解算指令的情况**，相当于对一块内存同时进行读写操作，因此我们会使用ROS提供的实时工具 `RealtimeBuffer` 来保证线程中的数据安全。`RealtimeBuffer` 类在定义在 `realtime_tools` 命名空间下，其实现主要依靠进程锁 `mutex` （值得一提的是该类的实现并不依赖ros）。

现在我们将上面关于线程中数据安全的讨论更加具象一点，仍以控制器设计为例子。

```c++
MyController::update(const ros::Time& time, const ros::Duration& period) 
{
    ....
    joint_vel_ = processData(data_callback_);   
    ....
}

/*
 * @brief : Subscriber调用的回调函数
*/
MyController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    data_callback_ = *msg;
}

/------------------------------- Thread - Safe Code ---------------------------------/
    
MyController::update(const ros::Time& time, const ros::Duration& period) 
{
    mutex_.lock();
    ....
    joint_vel_ = processData(data_callback_);   
    ....
    mutex_unlock();
}

/*
 * @brief : Subscriber调用的回调函数
*/
MyController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    mutex_.lock();
    data_callback_ = *msg;
    mutex_.unlock();
}
```

上述的第一部分程序是数据不安全的，我们可以简单粗暴地将其改写一下，得到第二部分程序，使其在不考虑线程阻塞问题的情况下保证数据安全。或许你说，通过修改进程锁开锁关锁的位置、使用 `try_lock()` 、增加额外判断逻辑可使得阻塞问题彻底消失。每次接收数据都要考虑这么多，这样开发太累了，幸运的是，[**RealtimeBuffer**](https://github.com/ros-controls/realtime_tools/blob/indigo-devel/include/realtime_tools/realtime_buffer ) 已经帮我们都想好了。

```c++
template <class T>
class RealtimeBuffer
{
 public:
  RealtimeBuffer()
    : new_data_available_(false)
  {
    // allocate memory
    non_realtime_data_ = new T();
    realtime_data_ = new T();
  }
    
  ......
  
  T* readFromRT()
  {
    // Check if the data is currently being written to (is locked)
    if (mutex_.try_lock())
    {
      // swap pointers
      if (new_data_available_)
      {
        T* tmp = realtime_data_;
	realtime_data_ = non_realtime_data_;
        non_realtime_data_ = tmp;
	new_data_available_ = false;
      }
      mutex_.unlock();
    }
    return realtime_data_;
  }
      
  ......
    
  void writeFromNonRT(const T& data)
  {
    // get lock
    lock();

    // copy data into non-realtime buffer
    *non_realtime_data_ = data;
    new_data_available_ = true;

    // release lock
    mutex_.unlock();
  }
};
```

摘出了 `RealtimeBuffer` 类中两个关键成员函数，`writeFromNonRT` 用于在非实时线程中存储数据，而 `readFromRT` 则用于在实时线程中读取数据。显而易见，在 `NonRT` 和 `RT` 间我们需要构造一个 `RealtimeBuffer` 对象用于作为数据传递的中介。通过一系列逻辑判断，该类使得我们在实时线程中总是能获取到非实时线程中最新一次收到的数据。现在运用该工具来修改我们的控制器。

```c++
MyController::update(const ros::Time& time, const ros::Duration& period) 
{
    ....
    auto temp_data = *(realtime_buffer_.readFromRT());
    joint_vel_ = processData(temp_data_);
    /* 以下写法应该也不会引起数据安全问题：
     * joint_vel_ = processData(*(realtime_buffer_.readFromRT()));
    */
    ....
}

/*
 * @brief : Subscriber调用的回调函数
*/
MyController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    realtime_buffer_.writeFromNonRT(*msg);
}
```



# C++语法 C++

## 驼峰命名法 Camel-Case

为了使代码具有可读性，我们设定若干规则：

1. `class` 和 `struct`的每个单词首字母都大写
2. `class` 内部的成员变量结尾增加下划线 `_`
3. 所有函数尽量以动词开头，并且第一个单词首字母小写，其余单词首字母大写
4. 一个声明为 `const` 的常量，首字母增加 `k`，其余单词首字母大写

```C++
int kDaysOfYear = 365;

...
    
class MyClass{
    public:
    int doSomething()
    {
        ...
    }
    private:
    std::string name_;
    int age_;
};
```



## 类与对象 Class and Object

### 类的构造 1  Constructor 1

类的构造是复杂的，且牵涉到相当多的细节，`类的构造`部分只讲述一些基础的内容、应该关注的细节、易错的语法问题，不会关注于移动语义、复制构造，引用构造、隐式构造的知识，也不会牵扯到继承与父类构造函数的联系。（以后的章节我会慢慢写，现阶段不用

我们从最简单的类开始。该类包含两个私有成员和两个公有成员函数。我们可以通过`setProperties()`为我们已经声明的对象内的成员赋值，再利用`readProperties()`读取成员变量的值。该类虽然简单但是也体现了类的封装特性，其他程序必须使用该类提供的公有部分的接口才能够对其内的成员变量进行读取和修改。

```c++
class Entity{
public:
    void setProperties(int x,int y){ 
        x_ = x;
		y_ = y;
    }
    void readProperties()
    { std::cout<<"x= "<<x_<<" y= "<<y_<<std::endl; }
private:
    int x_, y_;
}; //别忘了这里的分号

int main()
{
    Entity ent;
    ent.setProperties(1,2);
    ent.readProperties();
    return 0;
}
```

 现在让我们`F5`跑一下代码。

```c++
RESULT:
x= 1 y= 2
```

这个结果并不是很奇怪，很符合我们的预期。但是让我们思考一下，有没有更简单的办法对这个对象进行初始化，就像最基础的变量那样？答案是有的，现在改写我们的`Entity`，在类中增加一个没有返回值的与类同名的函数。（我们称其为构造函数）

```c++
class Entity{
public:
	Entity(int x, int y){ 
        x_ = x;
		y_ = y;
    }
	
    void setProperties(int x, int y){ 
        x_ = x;
		y_ = y;
    }
    void readProperties()
    { std::cout<<"x= "<<x_<<" y= "<<y_<<std::endl; }
private:
    int x_, y_;
};

int main()
{
    Entity ent = {1,2};
    ent.readProperties();
    return 0;
}
```

再让我们`F5`跑一下代码。结果与之前完全一样，完全没有问题。但是我们现在让他来出现一点问题。修改`main`函数为最开始的样子，然后编译。Oops，编译无法通过，问题出现在哪里，又或者说我们改变了什么？`Entity ent;`我们修改了不带参数的初始化语句。由于增加了带参数的构造函数，现在编译器不知道如何在不带参数的情况下初始化我们的对象了！OK，我们继续修改一下我们的类，如下：

```
class Entity{
public:
	Entity(){}
	Entity(int x, int y){ 
		x_ = x;
		y_ = y;
	}
	void setProperties(int x, int y){ 
		x_ = x;
		y_ = y;
	}
	void readProperties()
	{ std::cout<<"x= "<<x_<<" y= "<<y_<<std::endl; }
private:
	int x_, y_;
};
```

这次程序可以编译了，而且结果也与前几次相同。而我们做的仅仅是增加了一个不带参数的构造函数罢了（这个构造函数是空的）。或许你会疑问，刚开始一个构造函数都没有的时候，程序不是可以正常编译和运行吗？

事实上，C++的类，只有**在你没有定义任何构造函数的时候才会为你提供一个空构造函数**，一旦你自己写了任何一个构造函数，那么这个空构造函数就不会被提供了，必须你自己写（除非你每次都愿意带参构造，那么你可以不写）。

### 类的构造 2  Constructor 2

TODO : 初始化列表 、 深拷贝&浅拷贝 、 something else ？



## 一些C11特性  some C11 Feature

### Function 特性  Feature Function

作为引入部分，介绍一下函数指针。利用函数指针，我们可以将返回值数据类型和参数列表数据类型相同的函数归成一类。该功能使得我们可将函数作为参数传入其他函数。这段话或许有点抽象，但请阅读下述代码。我们首先定义了一个 `Person` 类，并提供一个名为 `doSomething` 的公有方法，该方法参数为一个无返回值且参数列表为空的函数（函数指针）。

```c++
#include <string>
#include <iostream>

class Student{
public:
	Person()=default;
	void doSomething(void (*something)(void));
};

void Student::doSomething(void (*something)(void))
{	something();  }

void something1()
{	std::cout << "摸鱼" << std::endl;   }
void something2()
{	std::cout << "摸更多鱼" << std::endl;   }

int main()
{
    Student you;
    you.doSomething(&something1); // = you.doSomething(something1);  --implicit--
    you.doSomething(&something2);
}
```

结果是显然的，如果不想摸鱼的话，可以传几个其他的函数传入 `doSomething(void(*something)(void))` 中。 现在我们可以建立一个概念：函数也可以作为参数传入其他函数中，而这一功能可以让程序更加的模块化。有了铺垫，现在正式介绍一下 `Function`  特性，可以简单地认为 `Function` 是一种更加安全且可读性更高的函数指针。此外，使用该特性可以更方便的指向类内成员函数。下面的程序分别使用函数指针和 `Function` 特性，其效果是 `useFunctionFeature` 和 `useFunctionPtr` 指向同一函数 `printNum`。总之，在不考虑系统开销的情况下，应尽量使用 `Function` 特性。

```c++
#include <functional>
#include <iostream>
#include <string>

void printNum(int num)
{
    std::cout << num << std::endl;
}

int main()
{
    std::function<void(int)> useFunctionFeature;
	void (*useFunctionPtr)(int);
    
    useFunctionFeature = &printNum;
    useFunctionPtr = &printNum;
    useFunctionFeature(1);
    useFunctionPtr(2);
}
```



### bind 特性  Feature bind

作为C++11的新特性，`bind` 和 `Function` 常常成对使用（意思是不成对使用也可以），在我们的项目中，使用这两个特性来完成回调函数的功能，如 `sentry_communicator` 包下的`socketcan.cpp` 和 `can_bus.cpp` 中就运用了这些特性，构建了用于CAN通讯的数据接收回调函数。

> 如果是在 windows 环境下写程序，且没有安装boost库，那么可以使用 std::bind 替代 boost::bind， 对于一般使用两者差别不大。

```c++
#include <iostream>
#include <string>
#include <functional>

class HardwareInterface {
public:
    HardwareInterface() = default;
    HardwareInterface(std::function<void(const int& data)> transmission_handler);
    void doTransmission(const int& data);
    std::function<void(const int& )> transmission_handler_;
};

HardwareInterface::HardwareInterface(std::function<void(const int& )> transmission_handler) {
    transmission_handler_ = transmission_handler;
}
void HardwareInterface::doTransmission(const int& data)
{
    printf("Start to Transmit ...\n");
    transmission_handler_(data);
}

class User {
public:
    std::string username_;

    User() = default;
    User(std::string username);
    void Transmission(const int& data)
    {
        hardware_interface_.doTransmission(data);
    }
private:
    HardwareInterface hardware_interface_;
    void doDataParse(const int& data);
};

User::User(std::string username) :username_(username)
{
    hardware_interface_.transmission_handler_= std::bind(&User::doDataParse, this, std::placeholders::_1);
}
void User::doDataParse(const int& data) {
    printf("Parsing Data ...\n");
}


int main() {
    User user("Lithesh");
    user.Transmission(4);
}
```

# 测试说明

## Gazebo测试

### 底盘测试

按顺序执行如下命令，在Gazebo中进行底盘测试。

打开Gazebo并在其中加载工程模型

```cpp
roslaunch sp_description gazebo.launch 
```

加载底盘控制器chassis_controller

```cpp
roslaunch chassis_controller chassis_controller_load.launch
```

接着加载键盘控制器keyboard_control

```
rosrun keyboard_control keyboard_control
```

选中加载keyboard_control的命令行界面。控制方式为：

W 前

S 后

A 左

D 右

Q 逆时针旋转

E 顺时针旋转

### 机械臂测试

## 实机测试

### 单电机测试

(WARNING 23/4/28 

2006/3508单电机未带负载时存在严重抖动问题，请勿进行实机单电机测试)

按照下述顺序，使用单自由度连杆模型进行单电机测试。

（1）修改xacro模型中的减速比参数

文件位于`sp_description/urdf/test/rmrobot.xarco `

```cpp
<actuator name="right_front_wheel_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
</actuator>
```

修改`<mechanicalReduction>`标签内的减速比。

各电机减速比：

```cpp
rm_3508: 19.2032
DM_J4310: 1
MG_8016: 6
```

并修改`<joint name="joint1" type="">`中`type`类型，转动关节电机为`revolute`，平动关节电机为`prismatic`。

（2）修改yaml文件

文件位于`sp_hw/config/hw_config.yaml `

```cpp
actuators:
  right_front_wheel_motor:
    bus: can0
    id: 0x201
    type: rm_3508
```

```cpp
actuators:
  right_front_wheel_motor:
    bus: can0
    id: 0x001
    type:DM_J4310
```

```cpp
actuators:
  right_front_wheel_motor:
    bus: can0
    id: 0x141
    type: MG_8016
```

根据所测电机类型修改yaml文件对应参数。

（3）开启测试程序

给电机上电，并如下顺序执行命令

```cpp
sudo ip link set can0 up type can bitrate 1000000
candump can0
```

如运行正常，会出现读取的`can`帧信息。
执行如下命令

```cpp
roslaunch sp_hw load_hw_test.launch
```

接着执行

```cpp
roslaunch chassis_controller load_controllers.launch
```

开启不同`controller`读取/执行操作

`joint_state_controller`:

```cpp
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/joint_state_controller']
stop_controllers: ['']
strictness: 1
start_asap: true
timeout: 0.0"

rostopic echo /joint_states
```

`joint_position_controller`:

```cpp
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/joint1_position_controller']
stop_controllers: ['controllers/joint1_velocity_controller']
strictness: 1
start_asap: true
timeout: 0.0"
    
rostopic pub /controllers/joint1_position_controller/command std_msgs/Float64 "data: 0.0"
```

更改发送给`data`的数据控制电机位置。

`joint_velocity_controller`:

```cpp
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/joint1_velocity_controller']
stop_controllers: ['controllers/joint1_position_controller']
strictness: 1
start_asap: true
timeout: 0.0"

rostopic pub /controllers/joint1_velocity_controller/command std_msgs/Float64 "data: 3.14159"
```

更改发送给`data`的数据控制电机转速。

### 底盘测试

按照下述顺序，使用工程模型进行底盘测试。

1）修改xacro模型

文件位于`sp_description/urdf/engineer/engineer.xarco `

```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="engineer">
	<!--<xacro:include filename="$(find sp_description)/urdf/engineer/engineer_manipulator/engineer_manipulator.xacro"/>-->
	<xacro:include filename="$(find sp_description)/urdf/engineer/engineer_chassis/engineer_chassis.xacro"/>
	<!--<xacro:include filename="$(find sp_description)/urdf/engineer/manipulator_transmission.xacro"/>
	<xacro:include filename="$(find sp_description)/urdf/engineer/fpv_module/fpv_module.xacro"/>-->
	<xacro:include filename="$(find sp_description)/urdf/common/lidar2d.xacro"/>

	<xacro:arg name="use_simulation" default="true"/>
	<xacro:engineer_chassis roller_type="simple"/>
	<!--<xacro:fpv_module/>
	<xacro:engineer_manipulator/>
	

	<xacro:lidar2d connected_to="base_link" lidar_name="rplidar_front"
				   simulation="$(arg use_simulation)"
				   xyz="0.22 0 0.165" rpy="0 0 0"/>
		
	<xacro:lidar2d connected_to="base_link" lidar_name="rplidar_back"
				   simulation="$(arg use_simulation)"
				   xyz="-0.22 0 0.165" rpy="0 0 3.1416"/>

	<joint name="joint_arm2body" type="fixed">
		<axis xyz="0 0 1"/>
		<origin xyz="0.022 0.166 0.228" rpy="0 0 -1.5708"/>
		<parent link="base_link"/>
		<child link="arm_base_link"/>
	</joint>
	<joint name="joint_mast2body" type="fixed">
			<axis xyz="0 0 1"/>
			<origin xyz="-0.2295 -0.0825 0.0177" rpy="0 0 1.5708"/>
			<parent link="base_link"/>
			<child link="fpv_base_link"/>
	</joint>-->

	<gazebo>
		<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
			<robotNamespace>/</robotNamespace>
		</plugin>
		<plugin name="chassis_controller" filename="libchassis_controller.so">
			<robotNamespace>/</robotNamespace>
		</plugin>
		<plugin name="gimbal_controller" filename="libgimbal_controller.so">
			<robotNamespace>/</robotNamespace>
		</plugin>
	</gazebo>
	
</robot>
```

只留下底盘轮组四个`transmission`

2）开启测试程序

#注意检查底盘电机及轮组可动部分是否碰到传输线

#在架高底盘或空旷场地情况下进行测试

如下顺序执行命令

```cpp
roslaunch sp_hw load_hw.launch
```

此时给电机上电，执行如下命令

```cpp
sudo ip link set can0 up type can bitrate 1000000
candump can0
```

如运行正常，会出现读取的`can`帧信息。

接着执行

```cpp
roslaunch chassis_controller chassis_controller_load.launch
```

开启`chassis_controller`

接着发送话题

```
rostopic pub -r 50 /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

可看到麦轮向后旋转。

改变该话题发送的数据以测试不同情况下底盘运动状况。

## 遥控器/键鼠控制

遥控器/键鼠采用串口dbus与miniPC进行同行。请将遥控器接收端连接C板或串口dbus板，之后接入miniPC调试。

执行如下命令

```cpp
sudo chmod 777 /dev/ttyUSB0
```

接着开启`roscore`（若已开启`roslaunch`则无需单开`roscore`），并执行如下命令开启dbus

```cpp
rosrun sp_dbus sp_dbus
```

接下来就可以使用遥控器/键鼠控制仿真/实体工程。

### 控制说明

#### 遥控器控制模式

遥控器右拨杆上档进入遥控器控制模式。

右摇杆前后控制前进后退。

右摇杆左右控制向左向右。

左摇杆左右控制逆时针顺时针旋转。

左摇杆前后控制图传抬高降低视角。

#### 键鼠控制模式

遥控器右拨杆中档进入键鼠控制模式。

W控制前进。

S控制后退。

A控制向左。

D控制向右

Q控制图传逆时针旋转。

E控制图传逆时针旋转。

鼠标向左控制逆时针旋转。

鼠标向右控制逆时针旋转。

鼠标向上控制图传抬高视角。

鼠标向上控制图传降低视角。

鼠标滚轮向前控制图传上升。

鼠标滚轮向后控制图传下降。

#### 停止模式

遥控器右拨杆下档进入停止模式，工程底盘锁定。

# 远程桌面(无需局域网):

## 向日葵远程桌面控制软件:
https://sunlogin.oray.com/download?categ=personal  
在服务端ubuntu和用户端上分别安装对应的软件。
## 使用时需要将ubuntu切换到xorg:
https://service.oray.com/question/11969.html
## 设置向日葵开机启动：
（1）将向日葵启动脚本添加到init.d中：  
https://learnku.com/docs/ubuntustudy/linux-service-management-how-to-set-boot-script-in-ubuntu  

（2）设置自动登录：  
在settings-users中开启自动登录。  

（3）设置xorg自动登录：  
在/etc/gdm3/custom.conf中，取消注释#WaylandEnable=false。  

**！** 开机后需要等1-2分钟才能连接上。  
**！** 由于nuc在hdmi接口闲置时不会正确渲染图像，因此在使用远程桌面调试时需要将nuc的hdmi接口连接 hdmi诱骗器 或者 一根hdmi线。
