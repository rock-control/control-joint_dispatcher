#include <boost/test/unit_test.hpp>
#include <joint_dispatcher/Dispatcher.hpp>

using namespace joint_dispatcher;

BOOST_AUTO_TEST_CASE(it_should_be_able_to_resolve_the_names)
{
    base::samples::Joints joints;
    joints.names.push_back("0");
    joints.names.push_back("1");
    joints.names.push_back("2");

    JointSelection sel;
    sel.byName.push_back("2");
    sel.byName.push_back("0");
    sel.resolveNames(joints);
    BOOST_REQUIRE_EQUAL(2, sel.byIndex.size());
    BOOST_REQUIRE_EQUAL(2, sel.byIndex[0]);
    BOOST_REQUIRE_EQUAL(0, sel.byIndex[1]);

}

