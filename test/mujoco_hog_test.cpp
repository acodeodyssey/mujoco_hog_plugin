/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Julian Leichert */

#include <gtest/gtest.h>

// Ignore static variables unused in this compilation unit
// TODO(dleins): Remove this after object oriented refactoring
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <mujoco_ros/mujoco_sim.h>

#pragma GCC diagnostic pop

#include <mujoco_ros/common_types.h>
#include <mujoco_ros_msgs/SetGeomPosition.h>
#include <mujoco_ros_msgs/SetSolverParameters.h>
#include <mujoco_ros_msgs/SetWeldConstraintParameters.h>
#include <mujoco_ros_hog/mujoco_ros_hog_plugin.h>
#include <std_srvs/SetBool.h>

#include <geometry_msgs/Pose.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_test_node");
	return RUN_ALL_TESTS();
}

namespace unit_testing {

class MujocoRosBaseFixture : public ::testing::Test
{
protected:
	std::shared_ptr<ros::NodeHandle> nh;
	MujocoSim::MujocoEnvPtr env;
	MujocoSim::mjDataPtr d;
	MujocoSim::mjModelPtr m;
	std::unique_ptr<std::thread> mj_thread;
	mujoco_ros_hog::MujocoHogPlugin *hogPlugin;

	virtual void SetUp()
	{
		nh.reset(new ros::NodeHandle("~"));
		// nh->setParam("unpause", true);
		nh->setParam("no_x", true);
		// nh->setParam("use_sim_time", true);

		std::string xml_path = ros::package::getPath("mujoco_hog") + "/assets/hog_world.xml";

		mj_thread = std::unique_ptr<std::thread>(new std::thread(MujocoSim::init, xml_path, "example_hash"));
		while (MujocoSim::detail::settings_.loadrequest.load() == 0) { // wait for request to be made
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		while (MujocoSim::detail::settings_.loadrequest.load() > 0) { // wait for model to be loaded
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		env = MujocoSim::detail::main_env_;
		d   = env->data_;
		m   = env->model_;
	}

	virtual void TearDown()
	{
		MujocoSim::requestExternalShutdown();
		mj_thread->join();
	}
};

TEST_F(MujocoRosBaseFixture, set_solver_parameters)
{
	std::vector<MujocoSim::MujocoPluginPtr> plugins;

	for (auto p : plugins) {
		hogPlugin = dynamic_cast<mujoco_ros_hog::MujocoHogPlugin *>(p.get());
		if (hogPlugin != nullptr) {
			ROS_INFO_STREAM("Plugin loaded");
		} else {
			return;
		}
	}
	nh->setParam("unpause", false);
	mujoco_ros_msgs::SetSolverParameters srv;
	ROS_INFO_STREAM("I'm running the first test now");
	// correct parameters
	srv.request.solverParameters.dampratio = 0.01;
	srv.request.solverParameters.timeconst = 1;
	srv.request.solverParameters.dmax      = 0.95;
	srv.request.solverParameters.env_id    = 0;
	srv.request.solverParameters.dmin      = 0.95;
	srv.request.solverParameters.midpoint  = 0.5;
	srv.request.solverParameters.width     = 0.001;
	srv.request.solverParameters.power     = 2;
	srv.request.solverParameters.name      = "hogBox";
	srv.request.admin_hash                 = "example_hash";
	hogPlugin->setSolverParametersCB(srv.request, srv.response);
	// wrong body name
	// mujoco_ros_hogMujocoHogPlugin::setSolverParametersCB(srv.request,srv.response);

	// wrong env_id
}

TEST_F(MujocoRosBaseFixture, set_eq_active)
{
	nh->setParam("unpause", false);
	std_srvs::SetBool srv;
	srv.request.data          = true;
	ros::ServiceClient client = nh->serviceClient<std_srvs::SetBool>("mujoco_ros_hog/set_eq_active");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";
}

} // namespace unit_testing
