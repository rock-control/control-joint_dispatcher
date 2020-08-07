#ifndef JOINT_DISPATCHER_INPUT_HPP
#define JOINT_DISPATCHER_INPUT_HPP

#include <string>
#include <base/samples/Joints.hpp>
#include <joint_dispatcher/SingleDispatch.hpp>

namespace joint_dispatcher
{
    /** Configuration of an input on Dispatcher
     */
    class Input
    {
        std::string mName;
        base::Time mLastUpdate;
        std::vector<std::string> inputJointNames;

    public:
        Input(std::string const& name = "");

        std::string getName() const;

        /** The list of dispatches that use this input
         */
        std::vector<SingleDispatch> dispatches;

        /** Write a new sample on this input
         */
        void write(base::samples::Joints const& sample);

        /** Resets the internal tracking state, without changing the
         * configuration
         */
        void reset();
    };
}

#endif

