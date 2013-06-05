#include <joint_dispatcher/Output.hpp>

using namespace joint_dispatcher;
using namespace std;

Output::Output(std::string const& name)
    : mName(name)
    , mIsNew(false) {}

std::string Output::getName() const
{
    return mName;
}

size_t Output::size() const
{
    return mState.size();
}

vector<string> Output::getJointNames() const
{
    return mState.names;
}

vector<size_t> Output::mapJointNamesToIndex(vector<string> const& names) const
{
    vector<size_t> result(names.size());
    for (size_t nameIdx = 0; nameIdx < names.size(); ++nameIdx)
        result[nameIdx] = mState.mapNameToIndex(names[nameIdx]);
    return result;
}

void Output::resize(vector<string> const& jointNames)
{
    resize(jointNames.size());
    mState.names = jointNames;
}

void Output::resize(size_t size)
{
    mState.resize(size);
    mUpdatedJoints.resize(size);
    mUpdateTime.resize(size);
    reset();
}

void Output::reset()
{
    fill(mUpdatedJoints.begin(), mUpdatedJoints.end(), false);
    fill(mUpdateTime.begin(), mUpdateTime.end(), base::Time());
    mFullUpdateCounter = mState.size();
}

bool Output::isNew() const
{
    return mIsNew;
}

bool Output::isFullyUpdated() const
{
    return mFullUpdateCounter == 0;
}

void Output::updateJoint(size_t jointIdx, base::Time const& time, base::JointState const& sample)
{
    mIsNew = true;
    mState.states[jointIdx] = sample;
    mUpdateTime[jointIdx] = time;
    if (!mUpdatedJoints[jointIdx])
    {
        mUpdatedJoints[jointIdx] = true;
        mFullUpdateCounter--;
    }
}

base::samples::Joints Output::read()
{
    fill(mUpdatedJoints.begin(), mUpdatedJoints.end(), false);
    mFullUpdateCounter = mUpdatedJoints.size();
    return mState;
}
