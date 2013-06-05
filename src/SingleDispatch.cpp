#include <joint_dispatcher/SingleDispatch.hpp>
#include <joint_dispatcher/Output.hpp>

using namespace joint_dispatcher;
using namespace std;

SingleDispatch::SingleDispatch()
    : output_channel(0) {}

void SingleDispatch::resolveInputNames(base::samples::Joints const& sample)
{
    input.resolveNames(sample);
}

void SingleDispatch::write(base::samples::Joints const& sample)
{
    for (size_t i = 0; i < input.size(); ++i)
    {
        size_t outputIdx = output.byIndex[i];
        output_channel->updateJoint(outputIdx, sample.time, sample.states[input.byIndex[i]]);
    }
}

