#ifndef JOINT_DISPATCHER_SINGLE_DISPATCH_HPP
#define JOINT_DISPATCHER_SINGLE_DISPATCH_HPP

#include <joint_dispatcher/JointSelection.hpp>
#include <base/Time.hpp>
#include <base/samples/Joints.hpp>

namespace joint_dispatcher
{
    class Output;

    /** Configuration of a single dispatch on Dispatcher
     */
    struct SingleDispatch
    {
        /** The joint selection on the input */
        JointSelection input;
        /** The output channel */
        Output* output_channel;
        /** The joint selection on the output */
        JointSelection output;

        SingleDispatch();

        void resolveInputNames(base::samples::Joints const& input_sample);

        void write(base::samples::Joints const& input);

        void reset();
    };
}

#endif
