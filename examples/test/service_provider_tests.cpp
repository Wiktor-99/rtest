// Copyright 2025 Spyrosoft Limited.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @file      service_provider_tests.cpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#include <gtest/gtest.h>

#include <test_composition/service_provider.hpp>

class ServiceProviderTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(ServiceProviderTest, WhenServiceRequestReceived_ThenStateIsUpdated)
{
}

TEST_F(ServiceProviderTest, WhenServiceIsLocked_ThenStateIsNotUpdated)
{
}