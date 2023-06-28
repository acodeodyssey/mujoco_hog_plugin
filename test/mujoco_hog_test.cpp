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

#include <mujoco_ros/mujoco_sim.h>
#include <mujoco_ros/mujoco_env.h>

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
		nh->setParam("unpause", true);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);

		std::string xml_path = ros::package::getPath("mujoco_hog") + "/assets/hog_world.xml";
		ROS_INFO_STREAM(xml_path);

		mj_thread = std::unique_ptr<std::thread>(new std::thread(MujocoSim::init, xml_path, ""));

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
	std::vector<MujocoSim::MujocoPluginPtr> plugins = env->getPlugins();

	for (auto p : plugins) {
		hogPlugin = dynamic_cast<mujoco_ros_hog::MujocoHogPlugin *>(p.get());
		if (hogPlugin != nullptr) {
			ROS_INFO_STREAM("Plugin loaded");
			break;
		} else {
			return;
		}
	}
	mujoco_ros_msgs::SetSolverParameters srv;
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
	EXPECT_TRUE(srv.response.success) << "Setting correct solver parameters failed";

	srv.request.solverParameters.dmax = 0;
	hogPlugin->setSolverParametersCB(srv.request, srv.response);
	EXPECT_FALSE(srv.response.success) << "Wrong response for wrong parameters";
	srv.request.solverParameters.name = "wrongboxname";
	hogPlugin->setSolverParametersCB(srv.request, srv.response);
	// wrong body name
	EXPECT_FALSE(srv.response.success) << "Wrong response for wrong body name";
}

TEST_F(MujocoRosBaseFixture, set_weld_constraints)
{
	std::vector<MujocoSim::MujocoPluginPtr> plugins = env->getPlugins();

	for (auto p : plugins) {
		hogPlugin = dynamic_cast<mujoco_ros_hog::MujocoHogPlugin *>(p.get());
		if (hogPlugin != nullptr) {
			ROS_INFO_STREAM("Plugin loaded");
			break;
		} else {
			return;
		}
	}
	mujoco_ros_msgs::SetWeldConstraintParameters srv;

	// empty parameters
	srv.request.admin_hash      = "";
	srv.request.set_solimp      = false;
	srv.request.set_solref      = false;
	srv.request.set_torquescale = false;
	hogPlugin->setWeldConstraintParametersCB(srv.request, srv.response);
	EXPECT_FALSE(srv.response.success) << "Empty parameters resulted in wrong response";
	srv.request.set_solimp                            = true;
	srv.request.set_solref                            = true;
	srv.request.set_torquescale                       = true;
	srv.request.parameters.torquescale                = 0;
	srv.request.parameters.active                     = true;
	srv.request.parameters.solverParameters.dampratio = 1;
	hogPlugin->setWeldConstraintParametersCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success) << "Wr parameters resulted in wrong response";
}

TEST_F(MujocoRosBaseFixture, set_geom_position)
{
	std::vector<MujocoSim::MujocoPluginPtr> plugins = env->getPlugins();

	for (auto p : plugins) {
		hogPlugin = dynamic_cast<mujoco_ros_hog::MujocoHogPlugin *>(p.get());
		if (hogPlugin != nullptr) {
			ROS_INFO_STREAM("Plugin loaded");
			break;
		} else {
			return;
		}
	}
	mujoco_ros_msgs::SetGeomPosition srv;

	// empty parameters
	srv.request.admin_hash   = "";
	srv.request.set_position = false;
	hogPlugin->setGeomPositionCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success) << "Parameters resulted in wrong response";
	srv.request.position.env_id             = 0;
	srv.request.position.name               = "wrongboxname";
	srv.request.position.pose.orientation.w = 1.0;
	srv.request.position.pose.orientation.x = 0.0;
	srv.request.position.pose.orientation.y = 0.0;
	srv.request.position.pose.orientation.z = 0.0;
	srv.request.position.pose.position.x    = 0.0;
	srv.request.position.pose.position.y    = 0.0;
	srv.request.position.pose.position.z    = 0.0;

	hogPlugin->setGeomPositionCB(srv.request, srv.response);
	EXPECT_FALSE(srv.response.success) << "Wrong bodyname parameter resulted in wrong response";
	srv.request.position.name = "hogBox";
	hogPlugin->setGeomPositionCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success) << "Wr parameters resulted in wrong response";
}
} // namespace unit_testing
