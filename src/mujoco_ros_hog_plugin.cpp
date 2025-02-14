/**
 * Software License Agreement (BSD 3-Clause License)
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
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
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
 */

/* Authors: Julian Leichert*/
#include <mujoco_ros_hog/mujoco_ros_hog_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>
#include <std_srvs/SetBool.h>

namespace mujoco_ros_hog {

bool MujocoHogPlugin::load(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
	// Check that ROS has been initialized
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_hog",
		                       "A ROS node for Mujoco has not been initialized, unable to load plugin.");
		return false;
	}

	ROS_ASSERT(rosparam_config_.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	if (!rosparam_config_.hasMember("hogBodies")) {
		ROS_ERROR_NAMED("mujoco_ros_hog", "MujocoRosHogPlugin expects a 'hogBodies' rosparam specifying at least "
		                                  "one body");
		return false;
	}
	ROS_ASSERT(rosparam_config_["hogBodies"].getType() == XmlRpc::XmlRpcValue::TypeStruct);

	XmlRpc::XmlRpcValue::iterator itr;

	for (itr = rosparam_config_["hogBodies"].begin(); itr != rosparam_config_["hogBodies"].end(); ++itr) {
		if (itr->second.hasMember("desiredPose")) {
			ROS_ASSERT(itr->second["desiredPose"].getType() == XmlRpc::XmlRpcValue::TypeArray);
			std::vector<double> pose;
			for (int32_t i = 0; i < itr->second["desiredPose"].size(); ++i) {
				ROS_ASSERT(itr->second["desiredPose"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				pose.push_back(itr->second["desiredPose"][i]);
			}
			ROS_INFO_STREAM_NAMED("mujoco_ros_hog", "Found hog object, with a defined position, named: " << itr->first);
			registerHog(m, itr->first, pose);
		} else {
			ROS_INFO_STREAM_NAMED("mujoco_ros_hog", "Found hog object with name: " << itr->first);
			ROS_WARN_STREAM_NAMED("mujoco_ros_hog",
			                      "MujocoRosHogPlugin didn't read a desired Position for the body: " << itr->first);
			registerHog(m, itr->first);
		}
	}

	this->m = m;
	this->d = d;
	tf_buffer_.reset(new tf2_ros::Buffer());
	service_servers_.push_back(node_handle_->advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
	    "mujoco_ros_hog/set_active", [&](auto &request, auto &response) {
		    active           = request.data;
		    response.success = true;
		    return true;
	    }));
	service_servers_.push_back(node_handle_->advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
	    "mujoco_ros_hog/set_eq_active", [&](auto &request, auto &response) {
		    changeEqualityConstraints("", request.data);
		    response.success = true;
		    return true;
	    }));
	service_servers_.push_back(node_handle_->advertiseService("mujoco_ros_hog/set_solver_parameters",
	                                                          &MujocoHogPlugin::setSolverParametersCB, this));
	service_servers_.push_back(node_handle_->advertiseService("mujoco_ros_hog/set_weld_constraint_parameters",
	                                                          &MujocoHogPlugin::setWeldConstraintParametersCB, this));

	service_servers_.push_back(
	    node_handle_->advertiseService("mujoco_ros_hog/set_geom_position", &MujocoHogPlugin::setGeomPositionCB, this));
	ROS_INFO_NAMED("mujoco_ros_hog", "Hog initialized");
	return true;
}

bool MujocoHogPlugin::setSolverParametersCB(mujoco_ros_msgs::SetSolverParameters::Request &req,
                                            mujoco_ros_msgs::SetSolverParameters::Response &resp)
{
	mjtNum solimp[5] = {};
	mjtNum solref[2] = {};
	solimp[0]        = req.solverParameters.dmin;
	solimp[1] = req.solverParameters.dmax, solimp[2] = req.solverParameters.width;
	solimp[3]    = req.solverParameters.midpoint;
	solimp[4]    = req.solverParameters.power;
	solref[0]    = req.solverParameters.timeconst;
	solref[1]    = req.solverParameters.dampratio;
	resp.success = setSolverParameters(req.solverParameters.name, solimp, solref);
	return true;
}

bool MujocoHogPlugin::setWeldConstraintParametersCB(mujoco_ros_msgs::SetWeldConstraintParameters::Request &req,
                                                    mujoco_ros_msgs::SetWeldConstraintParameters::Response &resp)
{
	bool weldResult  = setWeldConstraintParameters(req.parameters.solverParameters.name, req.parameters.active,
                                                 req.parameters.torquescale);
	mjtNum solimp[5] = {};
	mjtNum solref[2] = {};
	solimp[0]        = req.parameters.solverParameters.dmin;
	solimp[1] = req.parameters.solverParameters.dmax, solimp[2] = req.parameters.solverParameters.width;
	solimp[3]        = req.parameters.solverParameters.midpoint;
	solimp[4]        = req.parameters.solverParameters.power;
	solref[0]        = req.parameters.solverParameters.timeconst;
	solref[1]        = req.parameters.solverParameters.dampratio;
	bool solveResult = setSolverParameters(req.parameters.solverParameters.name, solimp, solref);
	resp.success     = (weldResult & solveResult);
	return true;
}

bool MujocoHogPlugin::setGeomPositionCB(mujoco_ros_msgs::SetGeomPosition::Request &req,
                                        mujoco_ros_msgs::SetGeomPosition::Response &resp)
{
	if (req.set_position) {
		mjtNum p[3] = { req.position.pose.position.x, req.position.pose.position.y, req.position.pose.position.z };
		mjtNum q[4] = { req.position.pose.orientation.w, req.position.pose.orientation.x, req.position.pose.orientation.y,
			             req.position.pose.orientation.z };
		resp.success = setPosition(req.position.name, p, q);
	} else {
		resp.success = true;
	}
	return true;
}

void MujocoHogPlugin::controlCallback(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
	if (d->time != last_time && active == true) {
		// updateHog(m, d);
	}
	last_time = d->time;
}

void MujocoHogPlugin::updateHog(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
	int env_id                  = 0;
	MujocoSim::MujocoEnvPtr env = MujocoSim::environments::getEnv(d.get());

	double g;
	// get the gravitational force
	if (env != nullptr) {
		g = env->model_->opt.gravity[2];
	} else {
		ROS_ERROR_STREAM_NAMED("mujoco_ros_hog", "Couldn't get Environment with ID: " << env_id);
		return;
	}
	// apply update for every registered hog body
	for (std::map<std::string, std::vector<double>>::iterator it = hog_bodies_.begin(); it != hog_bodies_.end(); ++it) {
		std::string body_name = it->first;
		geometry_msgs::TransformStamped hog_desired_tform;

		geometry_msgs::Vector3 pm;
		geometry_msgs::Quaternion qm;
		if (!it->second.empty() && it->second.size() == 7) {
			pm.x = it->second[0];
			pm.y = it->second[1];
			pm.z = it->second[2];
			qm.w = it->second[3];
			qm.x = it->second[4];
			qm.y = it->second[5];
			qm.z = it->second[6];

		} else {
			// uses transform if there is no desired position in the config
			try {
				hog_desired_tform = tf_buffer_->lookupTransform("hog_desired", body_name, ros::Time(0));
			} catch (tf2::TransformException ex) {
				ROS_ERROR_ONCE("%s", ex.what());
				return;
			}
			pm = hog_desired_tform.transform.translation;
			qm = hog_desired_tform.transform.rotation;
		}
		// retrieve id of object
		unsigned int fidx = mj_name2id(m.get(), mjOBJ_XBODY, body_name.c_str());
		unsigned int h;
		double kl = 2500;
		double ka = 250;
		double cl = 2.0 * sqrt(kl * m->body_subtreemass[fidx]);
		double ca = 2.0 * sqrt(ka * m->body_inertia[3 * fidx]);

		// get desired positions,quaternions as mujoco Nums
		mjtNum p[3]  = { pm.x, pm.y, pm.z };
		mjtNum q[4]  = { qm.w, qm.x, qm.y, qm.z };
		mjtNum pc[3] = { d->xpos[3 * fidx], d->xpos[3 * fidx + 1], d->xpos[3 * fidx + 2] };

		mjtNum qc[4] = { d->xquat[4 * fidx], d->xquat[4 * fidx + 1], d->xquat[4 * fidx + 2], d->xquat[4 * fidx + 3] };
		mju_normalize4(qc);
		mjtNum vx[6];
		mjtNum pe[3];
		// calculate difference between desired pos and actual pos
		mju_sub3(pe, p, pc);
		mj_objectVelocity(m.get(), d.get(), mjOBJ_XBODY, fidx, vx, 0);
		for (int i = 0; i < 3; ++i) {
			d->xfrc_applied[6 * fidx + i] = kl * pe[i] - cl * vx[i + 3];
		}
		d->xfrc_applied[6 * fidx + 2] += -1 * g * m->body_subtreemass[fidx];
		mjtNum ql[3];
		// calculate 3d velocity of difference between quats
		mju_subQuat(ql, q, qc);
		for (int i = 0; i < 3; ++i) {
			d->xfrc_applied[6 * fidx + i + 3] = ka * ql[i] - ca * vx[i];
		}
	}
}

void MujocoHogPlugin::changeEqualityConstraints(std::string bodyName, int eqActive)
{
	if (bodyName.empty()) {
		for (int i = 0; i < m->neq; i++) {
			m->eq_active[i] = eqActive;
		}
	} else {
		unsigned int fidx = mj_name2id(m.get(), mjOBJ_XBODY, bodyName.c_str());
		for (int i = 0; i < m->neq; i++) {
			if (fidx == m->eq_obj1id[i]) {
				m->eq_active[i] = eqActive;
				return;
			}
		}
	}
}

bool MujocoHogPlugin::setWeldConstraintParameters(std::string bodyName, bool active, double torqueScale)
{
	unsigned int fidx = mj_name2id(m.get(), mjOBJ_XBODY, bodyName.c_str());
	for (int i = 0; i < m->neq; i++) {
		if (fidx == m->eq_obj1id[i]) {
			m->eq_active[i]                = active;
			m->eq_data[i * mjNEQDATA + 10] = torqueScale;
			return true;
		}
	}
	return false;
}

bool MujocoHogPlugin::setSolverParameters(std::string bodyName, mjtNum solimp[5], mjtNum solref[2])
{
	unsigned int fidx = mj_name2id(m.get(), mjOBJ_XBODY, bodyName.c_str());
	if (solimp[1] < solimp[0] | solimp[4] < 1) {
		return false;
	}
	for (int i = 0; i < m->neq; i++) {
		if (fidx == m->eq_obj1id[i]) {
			for (int j = 0; j < 5; j++) {
				m->eq_solimp[i + j] = solimp[j];
				if (j < 2) {
					m->eq_solref[i + j] = solref[j];
				}
			}
			return true;
		}
	}
	return false;
}

void MujocoHogPlugin::updateEqRelPos(std::string bodyName, mjtNum p[3], mjtNum q[4])
{
	mjtNum quat1[4];
	mjtNum quat2[4];
	unsigned int fidx = mj_name2id(m.get(), mjOBJ_XBODY, bodyName.c_str());
	for (int i = 0; i < m->neq; i++) {
		if (fidx == m->eq_obj1id[i]) {
			int id2 = m->eq_obj2id[i];
			mju_negQuat(quat1, q); // neg(q0)
			mju_mulQuat(quat2, quat1, d->xquat + 4 * id2); // relpose = neg(q0) * q1;
			for (int j = 0; j < 4; j++) {
				if (j < 3) {
					m->eq_data[i * mjNEQDATA + j] = p[j];
				}
				m->eq_data[i * mjNEQDATA + j + 6] = quat2[j];
			}
		}
	}
}

bool MujocoHogPlugin::setPosition(std::string bodyName, mjtNum p[3], mjtNum q[4])
{
	int bodyid  = mj_name2id(m.get(), mjOBJ_BODY, bodyName.c_str());
	int qposadr = -1, qveladr = -1;

	// make sure we have a floating body: it has a single free joint
	if (bodyid >= 0 && m->body_jntnum[bodyid] == 1 && m->jnt_type[m->body_jntadr[bodyid]] == mjJNT_FREE) {
		// extract the addresses from the joint specification
		qposadr = m->jnt_qposadr[m->body_jntadr[bodyid]];
		if (qposadr != -1) {
			d->qpos[qposadr]     = p[0];
			d->qpos[qposadr + 1] = p[1];
			d->qpos[qposadr + 2] = p[2];
			d->qpos[qposadr + 3] = q[0];
			d->qpos[qposadr + 4] = q[1];
			d->qpos[qposadr + 5] = q[2];
			d->qpos[qposadr + 6] = q[3];
			updateEqRelPos(bodyName, p, q);
			return true;
		}
	}
	return false;
}

bool MujocoHogPlugin::registerHog(MujocoSim::mjModelPtr m, std::string bodyname, std::vector<double> desiredPose)
{
	int id = mj_name2id(m.get(), mjOBJ_XBODY, bodyname.c_str());
	// check if the body exists
	if (id == -1) {
		ROS_ERROR_STREAM_NAMED("mujoco_ros_hog", "Could not find hog body with the name:" << bodyname);
		return false;
	} else {
		ROS_INFO_STREAM_NAMED("mujoco_ros_hog", "Registered hog body with the name:" << bodyname);
		hog_bodies_.insert(std::pair<std::string, std::vector<double>>(bodyname, desiredPose));
	}
	return true;
}

void MujocoHogPlugin::reset(){};
} // namespace mujoco_ros_hog

PLUGINLIB_EXPORT_CLASS(mujoco_ros_hog::MujocoHogPlugin, MujocoSim::MujocoPlugin)
