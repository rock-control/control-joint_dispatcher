#include <boost/test/unit_test.hpp>
#include <joint_dispatcher/Dispatcher.hpp>

using namespace joint_dispatcher;

BOOST_AUTO_TEST_CASE(it_should_mark_as_new_only_when_all_joints_have_been_updated_at_least_once)
{
    Output out;
    out.resize(2);
    BOOST_REQUIRE(!out.isNew());
    out.updateJoint(0, base::Time(), base::JointState());
    BOOST_REQUIRE(!out.isNew());
    out.updateJoint(1, base::Time(), base::JointState());
    BOOST_REQUIRE(out.isNew());
    out.read();
    BOOST_REQUIRE(!out.isNew());
    out.updateJoint(1, base::Time(), base::JointState());
    BOOST_REQUIRE(out.isNew());
}

BOOST_AUTO_TEST_CASE(it_should_mark_as_fully_updated_only_if_all_joints_have_been_written)
{
    Output out;
    out.resize(2);
    BOOST_REQUIRE(!out.isFullyUpdated());
    out.updateJoint(0, base::Time(), base::JointState());
    BOOST_REQUIRE(!out.isFullyUpdated());
    out.updateJoint(1, base::Time(), base::JointState());
    BOOST_REQUIRE(out.isFullyUpdated());
}

BOOST_AUTO_TEST_CASE(it_should_not_mark_as_fully_updated_even_if_a_given_joint_is_written_more_than_once)
{
    Output out;
    out.resize(2);
    BOOST_REQUIRE(!out.isFullyUpdated());
    out.updateJoint(0, base::Time(), base::JointState());
    BOOST_REQUIRE(!out.isFullyUpdated());
    out.updateJoint(0, base::Time(), base::JointState());
    BOOST_REQUIRE(!out.isFullyUpdated());
}

BOOST_AUTO_TEST_CASE(it_should_reset_both_updated_flags_on_read)
{
    Output out;
    out.resize(2);
    out.updateJoint(0, base::Time(), base::JointState());
    out.updateJoint(1, base::Time(), base::JointState());
    BOOST_REQUIRE(out.isNew());
    BOOST_REQUIRE(out.isFullyUpdated());
    out.read();
    BOOST_REQUIRE(!out.isNew());
    BOOST_REQUIRE(!out.isFullyUpdated());
}

