rock_library(joint_dispatcher
    SOURCES Output.cpp Input.cpp SingleDispatch.cpp Dispatcher.cpp StateAggregator.cpp
    HEADERS Output.hpp Input.hpp SingleDispatch.hpp Dispatcher.hpp JointSelection.hpp StateAggregator.hpp
    DEPS_PKGCONFIG base-types)

