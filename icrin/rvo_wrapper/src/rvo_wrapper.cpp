/*
 * Example file showing a demo with 250 agents initially positioned evenly
 * distributed on a circle attempting to move to the antipodal position on the
 * circle.
 */

#define RVO_OUTPUT_TIME_AND_POSITIONS 0

#include <ros/ros.h>
#include <cmath>
#include <cstddef>
#include <vector>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#if _OPENMP
#include <omp.h>
#endif

#include <rvo_wrapper/RVO.h>
#include <rvo_wrapper/Definitions.h>

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;

size_t NAgents = 10;

void setupScenario(RVO::RVOSimulator* sim) {
	/* Specify the global time step of the simulation. */
	sim->setTimeStep(0.1f);

	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f);

	/*
	 * Add agents, specifying their start position, and store their goals on the
	 * opposite side of the environment.
	 */
	for (size_t i = 0; i < NAgents; ++i) {
		sim->addAgent(100.0f *
		              RVO::Vector2(std::cos(i * 2.0f * M_PI / 250.0f),
		                           std::sin(i * 2.0f * M_PI / 250.0f)));
		goals.push_back(-sim->getAgentPosition(i));
	}
}

#if RVO_OUTPUT_TIME_AND_POSITIONS
void updateVisualization(RVO::RVOSimulator* sim) {
	/* Output the current global time. */
	ROS_INFO("SimTime:%f", sim->getGlobalTime());

	/* Output the current position of all the agents. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		DEBUG(" " << sim->getAgentPosition(i));
	}
	DEBUG(std::endl);
}
#endif

void setPreferredVelocities(RVO::RVOSimulator* sim) {
	/*
	 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
	 * direction of the goal.
	 */
#ifdef _OPENMP
	#pragma omp parallel for
#endif
	for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}

		sim->setAgentPrefVelocity(i, goalVector);
	}
}

bool reachedGoal(RVO::RVOSimulator* sim) {
	/* Check if all agents have reached their goals. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > sim->getAgentRadius(
		      i) * sim->getAgentRadius(i)) {
			return false;
		}
	}

	return true;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "rvo_wrapper");
	ros::NodeHandle nh("rvo_wrapper");

	/* Create a new simulator instance. */
	RVO::RVOSimulator* sim = new RVO::RVOSimulator();

	/* Set up the scenario. */
	setupScenario(sim);

	/* Perform (and manipulate) the simulation. */
	ROS_INFO("Iteration");
	do {
#if RVO_OUTPUT_TIME_AND_POSITIONS
		updateVisualization(sim);
#endif
		setPreferredVelocities(sim);
		sim->doStep();
	} while (!reachedGoal(sim));
	ROS_INFO("End");

	delete sim;

	return 0;
}
