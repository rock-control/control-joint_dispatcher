#include <joint_dispatcher/Output.hpp>

using namespace joint_dispatcher;
using namespace std;

Output::Output(std::string const& name)
    : mName(name)
    , mIsNew(false)
    , mFullyInitialized(false) {}

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
    mFullyInitialized = false;
    mIsNew = false;
}

bool Output::isNew() const
{
    return mFullyInitialized && mIsNew;
}

bool Output::isFullyUpdated() const
{
    return mFullUpdateCounter == 0;
}

void Output::updateJoint(size_t jointIdx, base::Time const& time, base::JointState const& sample, bool needsRead)
{
    if (jointIdx >= size())
        throw std::out_of_range("given joint index is out of bounds");

    mIsNew = needsRead;
    mState[jointIdx] = sample;
    if (mState.time < time)
        mState.time = time;
    mUpdateTime[jointIdx] = time;
    if (!mUpdatedJoints[jointIdx])
    {
        mUpdatedJoints[jointIdx] = true;
        mFullUpdateCounter--;
        if (mFullUpdateCounter == 0)
            mFullyInitialized = true;
    }
}

base::samples::Joints Output::read()
{
    mIsNew = false;
    fill(mUpdatedJoints.begin(), mUpdatedJoints.end(), false);
    mFullUpdateCounter = mUpdatedJoints.size();
    return mState;
}
