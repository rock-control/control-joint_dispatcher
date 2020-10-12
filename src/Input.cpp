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
    // Never written or names changed, resolve names on the dispatches
    if ( mLastUpdate.isNull() || 
        inputJointNames.size() != joints.names.size() ||
        inputJointNames != joints.names ) 
    {
        for (size_t i = 0; i < dispatches.size(); ++i)
            dispatches[i].resolveInputNames(joints);
        
        inputJointNames = joints.names;
    }

    for (size_t i = 0; i < dispatches.size(); ++i)
        dispatches[i].write(joints);

    mLastUpdate = joints.time;
}

void Input::reset()
{
    mLastUpdate = base::Time();
}

