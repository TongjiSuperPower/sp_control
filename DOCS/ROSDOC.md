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





# 参数服务器 Parameter Server

ROS中使用参数服务器存储用户加载的参数。他支持多种数据类型，例如：整型、字符串、浮点数、布尔、字典、数组。

roscpp的参数服务器类API支持上述的所有数据类型，但不同数据类型API的使用难度并不一致，显然基础类型的数据类型（**整形、布尔、字符串、浮点**）使用起来会较为方便，而诸如**字典、数组**这样的组合类型则还需要借助**`XmlRpc::XmlRpcValue Class`**（before ROS Fuerte）或者`std::map`进行实现。

目前`std::map`支持的参数类型只有`bool、float、double、int、string`，因此如果要使用多级字典（比如<string,map>）则仍只能使用`XmlRpcValue`类型进行解析和检索。

TODO：去测试一下！

## XMLRPC



## 参数上载 Param Upload

参数上载可分为两步：编制 和 上传。

编制：我们经常使用一种有层次的结构声明数据，也即是YAML格式语言。参看下述以YAML格式书写的电机参数文件（config）。该文件在工程中用于定义电机的各项参数。可以看到在actuator_coefficient中我们定义了RM比赛中常用的两个电机的参数。

>  注：act2effort参数是使用电流转矩系数近似得出的，其真实情况应是非线性的，但以平衡步兵的控制情况来看，该非线性误差可以不考虑，故不深究。

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

上传：在launch文件中使用`<rosparam>`标签 。`<rosparam>` 标记也可以放在`node`标记内，在这种情况下，参数命名空间使用`Private Name`解析。

