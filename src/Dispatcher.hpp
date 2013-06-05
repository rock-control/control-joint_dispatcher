#ifndef JOINT_DISPATCHER_DISPATCHER_HPP
#define JOINT_DISPATCHER_DISPATCHER_HPP

#include <vector>
#include <joint_dispatcher/Input.hpp>
#include <joint_dispatcher/Output.hpp>

namespace joint_dispatcher
{
    /** Implementation of mux/demux logic for base::samples::Joints
     *
     * This class allows to transfer arbitrary sets of joints on inputs to other
     * sets on outputs. It allows to 
     */
    class Dispatcher
    {
        std::vector<Input>  mInputs;
        std::vector<Output> mOutputs;

    public:
        typedef size_t ChannelID;

        /** This declares an input to the dispatcher
         *
         * @param name the input name. This is mostly used for debugging /
         *   status purposes
         * @return the ID of the created input
         */
        ChannelID addInput(std::string const& name = "");

        /** This declares an output to the dispatcher, as well as the joint names
         * for this output
         *
         * @param name the output name. This can be left empty.
         */
        ChannelID addOutput(std::string const& name, std::vector<std::string> const& jointNames);

        /** This declares an output to the dispatcher, as well as the expected
         * number of joints.
         *
         * If non-zero, the number of joints is used for (1) consistency
         * checking and (2) pre-allocating the needed arrays.
         */
        ChannelID addOutput(std::string const& name = "", size_t size = 0);

        /** Returns the ID of an input from its name
         */
        ChannelID getInputByName(std::string const& name) const;

        /** Returns the ID of an output from its name
         */
        ChannelID getOutputByName(std::string const& name) const;

        /** This declares a mapping from some joints on a given input and some
         * joints on a given output
         */
        void addDispatch(
                ChannelID input,  JointSelection const& inputJoints,
                ChannelID output, JointSelection const& outputJoints);

        /** @overload
         */
        void addDispatch(std::string const& input,
                JointSelection const& inputJoints,
                std::string const& output,
                JointSelection const& outputJoints);

        /** Pushes a sample on a given input
         *
         * The outputs for which there is a dispatch are going to be updated
         */
        void write(ChannelID input,
                base::samples::Joints const& sample);

        /** @overload
         */
        void write(std::string const& input,
                base::samples::Joints const& sample);

        /** Reads a given output channel
         *
         * @return true if this channel had been updated since the last read,
         *   and false otherwise. The sample is not modified if the channel was
         *   not updated
         */
        bool read(ChannelID output, base::samples::Joints& sample);

        /** @overload
         */
        bool read(std::string const& name, base::samples::Joints& sample);
    };
}

#endif

