rock_library(joint_dispatcher
    SOURCES Output.cpp Input.cpp SingleDispatch.cpp Dispatcher.cpp
    HEADERS Output.hpp Input.hpp SingleDispatch.hpp Dispatcher.hpp JointSelection.hpp
    DEPS_PKGCONFIG base-types)

