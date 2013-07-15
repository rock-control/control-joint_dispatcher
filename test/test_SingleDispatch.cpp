#include <boost/test/unit_test.hpp>
#include <joint_dispatcher/Output.hpp>
#include <joint_dispatcher/SingleDispatch.hpp>

using namespace joint_dispatcher;

BOOST_AUTO_TEST_CASE(it_should_be_able_to_resolve_the_input_names)
{
}

BOOST_AUTO_TEST_CASE(it_should_resolve_the_input_names_on_first_write)
{
}

BOOST_AUTO_TEST_CASE(it_should_dispatch_an_input_sample_on_the_output)
{
    SingleDispatch dispatch;
    Output output;
    output.resize(3);
    dispatch.input.byIndex.push_back(0);
    dispatch.input.byIndex.push_back(2);
    dispatch.input.byIndex.push_back(1);
    dispatch.output_channel = &output;
    dispatch.output.byIndex.push_back(1);
    dispatch.output.byIndex.push_back(2);
    dispatch.output.byIndex.push_back(0);
    base::samples::Joints sample;
    sample.resize(3);
    sample[0].position = 0;
    sample[1].position = 1;
    sample[2].position = 2;
    dispatch.write(sample);

    sample = output.read();
    BOOST_REQUIRE_EQUAL(1, round(sample[0].position));
    BOOST_REQUIRE_EQUAL(0, round(sample[1].position));
    BOOST_REQUIRE_EQUAL(2, round(sample[2].position));
}


