
DEFINES += ROS_INTERFACE_NODE

ROS_INTERFACE_CFLAGS = $$system(rospack export --lang=cpp --attrib=cflags graspit_interface)
QMAKE_CXXFLAGS += $$ROS_INTERFACE_CFLAGS

ROS_INTERFACE_LFLAGS = $$system(rospack export --lang=cpp --attrib=lflags graspit_interface)
QMAKE_LFLAGS += $$ROS_INTERFACE_LFLAGS
