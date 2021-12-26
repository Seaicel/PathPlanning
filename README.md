# PathPlanning
控制小车寻路


# 使用方法
在创建好ros包并设置完成msg信息后，执行以下命令
```bash
find *.py |xargs cp -t ~/catkin_ws/<name>/scripts # name替换为创建的ros包的名称，如果不存在scripts目录，则创建该目录
#创建符号链接，progect_path替换为项目路径
cd ~/catkin_ws/devel/lib/<name>
ln -s <project_path>/DstarLite.py
ln -s <project_path>/LocalTarget.py
ln -s <project_path>/octmap.py
ln -s <project_path>/surface.py
```
修改~/catkin_ws/src/\<name\>/CMakeLists.txt，加入
```
catkin_install_python(PROGRAMS scripts/planning.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
之后就可以运行该ros节点
```bash
cd ~/catkin_ws
catkin_make
roscore
rosrun <name> planning.py #在新窗口运行
```

# 自定义msg

1. 将自定义msg复制到~/catkin_ws/src/\<name\>/msg目录下
2. 取消注释add_message_file(...)，并将新msg填入
3. 取消注释generate_messages，填入依赖msg包

