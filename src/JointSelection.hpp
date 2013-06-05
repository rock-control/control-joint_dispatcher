#ifndef JOINT_DISPATCHER_JOINT_SELECTION_HPP
#define JOINT_DISPATCHER_JOINT_SELECTION_HPP

#include <vector>
#include <string>
#include <base/samples/Joints.hpp>

namespace joint_dispatcher
{
    /** This represents a subset of joints from a base::samples::Joints
     */
    struct JointSelection
    {
        std::vector<int> byIndex;
        std::vector<std::string> byName;

        size_t size() const { return byIndex.size(); }

        void resolveNames(base::samples::Joints const& sample)
        {
            byIndex.resize(byName.size());
            for (size_t i = 0; i < byName.size(); ++i)
                byIndex[i] = sample.mapNameToIndex(byName[i]);
        }
    };
}

#endif

