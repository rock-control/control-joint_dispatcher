rock_library(joint_dispatcher
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(joint_dispatcher_bin Main.cpp
    DEPS joint_dispatcher)

