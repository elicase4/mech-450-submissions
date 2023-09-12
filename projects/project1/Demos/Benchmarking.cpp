/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Zachary Kingston, Ioan Sucan */

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>

using namespace ompl;

void benchmark0(std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    benchmark_name = std::string("Home");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(252.95);
    start->setY(-214.95);
    start->setZ(46.19);
    start->rotation().setAxisAngle(0., 0., -1., 3.12413936107);

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(-320.00);
    goal->setY(75.05);
    goal->setZ(46.19);
    goal->rotation().setAxisAngle(0., 0., -1., 3.12413936107);

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 325.00);
    bounds.setHigh(1, 337.89);
    bounds.setHigh(2, 146.19);
    bounds.setLow(0, -383.80);
    bounds.setLow(1, -371.47);
    bounds.setLow(2, -0.20);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 60.0*10.0;
    memory_limit = 100000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;
}

void benchmark1(std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    benchmark_name = std::string("Twistycool");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.);
    start->setY(160.);
    start->setZ(-200.);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(270.);
    goal->setY(160.);
    goal->setZ(-400.);
    goal->rotation().setIdentity();

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 350.);
    bounds.setHigh(1, 250.);
    bounds.setHigh(2, -150.);
    bounds.setLow(0, 200.);
    bounds.setLow(1, 75.);
    bounds.setLow(2, -450.);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 60.0*10.0;
    memory_limit = 100000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;
}


void benchmark2(std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    benchmark_name = std::string("Abstract");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Abstract_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Abstract_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(84.98);
    start->setY(-60.00);
    start->setZ(180.16);
    start->rotation().setAxisAngle(1., 0., 0., 0);

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(-121.02);
    goal->setY(12.00);
    goal->setZ(153.16);
    goal->rotation().setAxisAngle(1., 0., 0., 1.57079632679);

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 239.92);
    bounds.setHigh(1, 250.74);
    bounds.setHigh(2, 468.98);
    bounds.setLow(0, -233.12);
    bounds.setLow(1, -222.20);
    bounds.setLow(2, -3.95);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 60.0*10.0;
    memory_limit = 100000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;
}

int main(int argc, char **argv)
{
    app::SE3RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

    if (argc < 2)
        return -1;

    int benchmark_id = std::atoi(argv[1]);

    if (benchmark_id == 0)
        benchmark0(benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (benchmark_id == 1)
        benchmark1(benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (benchmark_id == 2)
        benchmark2(benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else
        return -1;

    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    auto rrtc1 = std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation());
    rrtc1->setName("RRTConnect Range 5.");
    rrtc1->setRange(5.);

    auto rrtc2 = std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation());
    rrtc2->setName("RRTConnect Range 25.");
    rrtc2->setRange(25.);

    auto rrtc3 = std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation());
    rrtc3->setName("RRTConnect Range 50.");
    rrtc3->setRange(50.);

    auto rrt1 = std::make_shared<geometric::RRT>(setup.getSpaceInformation());
    rrt1->setName("RRT Range 5.");
    rrt1->setRange(5.);

    auto rrt2 = std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation());
    rrt2->setName("RRT Range 25.");
    rrt2->setRange(25.);

    auto rrt3 = std::make_shared<geometric::RRT>(setup.getSpaceInformation());
    rrt3->setName("RRT Range 50.");
    rrt3->setRange(50.);

    b.addPlanner(rrtc1);
    b.addPlanner(rrtc2);
    b.addPlanner(rrtc3);
    b.addPlanner(rrt1);
    b.addPlanner(rrt2);
    b.addPlanner(rrt3);
    b.addPlanner(std::make_shared<geometric::KPIECE1>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile();

    return 0;
}
