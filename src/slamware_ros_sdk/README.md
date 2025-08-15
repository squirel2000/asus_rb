# slamware_ros_sdk(ros2 version)

# build

假如 sdk 工程在 home 目录: ~/sdk , 假如是 x86_64 架构,

1, 生成源码和对应的库和头文件

```shell
cd ~/sdk
./scripts/build_ros_sdk.sh x86_64
```

执行完脚本后，~/sdk目录下会有 build 和 output 目录, 其中output目录是输出的源码工程和对应的 头文件和库 压缩文件。

output下的两个压缩文件是从build目录拷贝的

2, 编译源码生成 ros2 node

加压缩第一步中生成的压缩文件并编译。

```shell
cd ~/sdk/output
tar zxvf slamware_ros_sdk_linux-x86_64-gcc9.tar.gz
cd slamware_ros_sdk_linux-x86_64-gcc9/src/slamware_ros_sdk
colcon build
source install/setup.bash
```

# run

**type 1**

> 这种方式暂时是硬编码 ip, port。

```
ros2 run slamware_ros_sdk slamware_ros_sdk_server_node
```


**type 2**
```
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml
```
> 在`编译源码生成 ros2 node`步骤中，进入`colcon build`目录，然后`cd launch`, 修改该目录下的`slamware_ros_sdk_server_node.xml` 文件的`<arg name="ip_address" default="192.168.11.1" />`行指定的ip，就是目标机器的IP，`colcon build`编译完直接启动即可，如果中途目标ip更改，可以再进入`colcon build`目录，然后`cd install/slamware_ros_sdk/share/slamware_ros_sdk/launch`,修改该目录下的`slamware_ros_sdk_server_node.xml` 文件的`<arg name="ip_address" default="192.168.11.1" />`行指定的ip运行node即可。

# log 

`log`路径：`$ROS_HOME/log`, 可设置 `$ROS_HOME` 变量的值,
`$ROS_HOME`默认值: `~/.ros`

xml 中  node 标签的 output 属性可设置log输出方式: "screen", "log" or "both"
如：
```
<node pkg="move_base" exec="move_base" name="move_base" output="both">
```
则设置log同时输出到屏幕和log文件。
