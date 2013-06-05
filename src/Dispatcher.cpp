#include <joint_dispatcher/Dispatcher.hpp>

using namespace joint_dispatcher;

Dispatcher::ChannelID Dispatcher::addInput(std::string const& name)
{
    mInputs.push_back(Input(name));
    return mInputs.size() - 1;
}

Dispatcher::ChannelID Dispatcher::addOutput(std::string const& name, size_t size)
{
    mOutputs.push_back(Output(name));
    mOutputs.back().resize(size);
    return mInputs.size() - 1;
}

Dispatcher::ChannelID Dispatcher::addOutput(std::string const& name, std::vector<std::string> const& jointNames)
{
    mOutputs.push_back(Output(name));
    mOutputs.back().resize(jointNames);
    return mInputs.size() - 1;
}

Dispatcher::ChannelID Dispatcher::getInputByName(std::string const& name) const
{
    for (size_t i = 0; i < mInputs.size(); ++i)
        if (mInputs[i].getName() == name)
            return i;
    throw std::runtime_error("there is no declared input named " + name);
}

Dispatcher::ChannelID Dispatcher::getOutputByName(std::string const& name) const
{
    for (size_t i = 0; i < mOutputs.size(); ++i)
        if (mOutputs[i].getName() == name)
            return i;
    throw std::runtime_error("there is no declared output named " + name);
}

void Dispatcher::addDispatch(
        ChannelID input,  JointSelection const& inputJoints,
        ChannelID output, JointSelection const& outputJoints)
{
    if (input >= mInputs.size())
        throw std::out_of_range("given input channel ID is out of bounds");
    if (output >= mInputs.size())
        throw std::out_of_range("given output channel ID is out of bounds");

    SingleDispatch dispatch;
    dispatch.input = inputJoints;
    dispatch.output_channel = &mOutputs[output];
    dispatch.output = outputJoints;
    // The names should be resolv-able right now
    dispatch.output.resolveNames(mOutputs[output].read());
    mInputs[input].dispatches.push_back(dispatch);
}

void Dispatcher::addDispatch(
        std::string const& input, JointSelection const& inputJoints,
        std::string const& output, JointSelection const& outputJoints)
{
    return addDispatch(getInputByName(input), inputJoints,
            getOutputByName(output), outputJoints);
}

void Dispatcher::write(ChannelID input, base::samples::Joints const& sample)
{
    if (input >= mInputs.size())
        throw std::out_of_range("given input channel ID is out of bounds");

    mInputs[input].write(sample);
}

void Dispatcher::write(std::string const& input, base::samples::Joints const& sample)
{
    write(getInputByName(input), sample);
}

bool Dispatcher::read(ChannelID output, base::samples::Joints& sample)
{
    if (output >= mOutputs.size())
        throw std::out_of_range("given output channel ID is out of bounds");

    if (mOutputs[output].isNew())
    {
        sample = mOutputs[output].read();
        return true;
    }
    else
    {
        return false;
    }
}

bool Dispatcher::read(std::string const& name, base::samples::Joints& sample)
{
    return read(getOutputByName(name), sample);
}

void Dispatcher::reset()
{
    for (size_t i = 0; i < mInputs.size(); ++i)
        mInputs[i].reset();
    for (size_t i = 0; i < mInputs.size(); ++i)
        mOutputs[i].reset();
}

