#include <joint_dispatcher/Input.hpp>

using namespace joint_dispatcher;

Input::Input(std::string const& name)
    : mName(name) {}

std::string Input::getName() const
{
    return mName;
}

void Input::write(base::samples::Joints const& joints)
{
    if (mLastUpdate.isNull()) // Never written, resolve names on the dispatches
    {
        for (size_t i = 0; i < dispatches.size(); ++i)
            dispatches[i].resolveInputNames(joints);
    }

    for (size_t i = 0; i < dispatches.size(); ++i)
        dispatches[i].write(joints);

    mLastUpdate = joints.time;
}

void Input::reset()
{
    mLastUpdate = base::Time();
}

