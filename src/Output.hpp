#ifndef JOINT_DISPATCHER_OUTPUT_HPP
#define JOINT_DISPATCHER_OUTPUT_HPP

#include <vector>
#include <string>
#include <base/Time.hpp>
#include <base/samples/Joints.hpp>

namespace joint_dispatcher
{
    /** Configuration of an output on Dispatcher
     */
    class Output
    {
        /** The output name
         */
        std::string mName;

        base::samples::Joints mState;

        /** If true, this output should be exported the next time
         * Dispatcher::read is called for it
         */
        bool mIsNew;
        /** A count of joints that need to be updated before the joint has been
         * fully updated
         */
        size_t mFullUpdateCounter;
        /** A joint-by-joint flag marking the joints as having been updated
         * since the last call to Dispatcher::read or not
         */
        std::vector<bool> mUpdatedJoints;
        /** A joint-by-joint update flag
         */
        std::vector<base::Time> mUpdateTime;

    public:
        Output(std::string const& name = "");

        std::string getName() const;

        /** Updates a joint on this output */
        void updateJoint(size_t jointIdx, base::Time const& time, base::JointState const& sample);

        /** Read the current joint state, resetting all the 'new' flags */
        base::samples::Joints read();

        /** The list of joint names */
        std::vector<std::string> getJointNames() const;

        /** Returns the list of indexes that correspond to the requested names
         *
         * @throws base::samples::Joints::InvalidName if one of the names does not exist
         */
        std::vector<size_t> mapJointNamesToIndex(std::vector<std::string> const& names) const;

        /** The count of joints */
        size_t size() const;

        /** Resize while setting the list of joint names at the same time */
        void resize(std::vector<std::string> const& jointNames);

        /** Declares the number of joints this output will generate */
        void resize(size_t size);

        /** Resets the internal tracking state to pristine, without changing
         * configuration
         */
        void reset();

        /** Returns true if the current sample is new, i.e. if it has been
         * updated since the last call to read()
         */
        bool isNew() const;

        /** Returns true if all joints of the current sample have been updated
         * since the last call to read()
         */
        bool isFullyUpdated() const;
    };
}

#endif

