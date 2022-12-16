#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "cb_hw_test/cb_hw_test.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

namespace cb_hw_test
{

CbHwTest::CbHwTest()
{
}

CbHwTest::~CbHwTest()
{
}



}  // namespace cb_hw_test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cb_hw_test::CbHwTest, hardware_interface::SystemInterface)
