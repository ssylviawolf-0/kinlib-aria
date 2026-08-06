// This file is combining example.cpp and buildWall.cpp in order to get Project
// Aria to work So we can get the screw trajectory And motion plan But

// This is currently done for the panda robot
#include <Eigen/Dense>
#include <array>
#include <cstdlib> // Required for rand() and srand()
#include <ctime>
#include <fstream>
#include <iostream>
#include <kinlib/kinlib_kinematics.h>
#include <kinlib/kinlib_resources.h>
#include <kinlib/motion_planning.h>

template <typename M> M loadCSV(const std::string &path) {
  std::ifstream indata;
  indata.open(path);
  std::cerr << "check" << std::endl;
  std::string line;
  std::vector<double> values;
  uint rows = 0;
  // if (std::getline(indata, line)) {

  // }

  while (std::getline(indata, line)) {
    if (line.empty() ||
        line.find_first_not_of(" \t\r\n") == std::string::npos) {
      continue;
    }
    std::stringstream lineStream(line);
    std::string cell;
    bool isRow = true;
    while (std::getline(lineStream, cell, ',')) {
      try {
        values.push_back(std::stod(cell));
      } catch (const std::invalid_argument &e) {
        std::cerr << " Couldn't convert line to number " << cell << std::endl;
        isRow = false;
      } catch (const std::out_of_range &e) {
        std::cerr << "number out of range: " << cell << std::endl;
        isRow = false;
      }
    }
    if (isRow == true) {
      rows++;
    }
  }
  indata.close();
  uint cols = values.size() / rows;
  std::cerr << rows << '&' << cols << std::endl;

  M matrix_data =
      Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime,
                                     M::ColsAtCompileTime, Eigen::RowMajor>>(
          values.data(), rows, cols);
  // return Eigen::Map<const Eigen::Matrix<typename M::Scalar,
  // M::RowsAtCompileTime, //////M::ColsAtCompileTime,
  // Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
  return matrix_data;
}

void saveGuidingPosesRaw(const std::string &path,
                         const std::vector<Eigen::Matrix4d> &goals) {
  std::ofstream ofs(path, std::ios::trunc);
  for (const auto &T : goals) {
    for (int r = 0; r < 4; ++r) {
      ofs << T(r, 0) << "," << T(r, 1) << "," << T(r, 2) << "," << T(r, 3)
          << "\n";
    }
  }
}

void saveGripperColRaw(const std::string &path, const std::vector<int> &grips) {
  std::ofstream ofs(path, std::ios::trunc);
  for (size_t k = 0; k < grips.size(); ++k) {
    ofs << grips[k] << "\n";
  }
}

void makeDummyGripperCSV(const std::string &newfilename,
                         const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return;
  }
  std::string header;
  std::getline(file, header);
  std::string line;
  int rowCount = 0;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    bool isOnlyWhitespace =
        std::all_of(line.begin(), line.end(),
                    [](unsigned char c) { return std::isspace(c); });

    if (!line.empty() && !isOnlyWhitespace) {
      rowCount++;
    }
  }
  file.close();
  std::cerr << rowCount << std::endl;

  std::ofstream newfile(newfilename);
  if (!newfile.is_open()) {
    std::cerr << "Error: Could not open file " << newfilename << std::endl;
    return;
  }
  for (int j = 0; j < (rowCount - 1) / 4; ++j) {
    newfile << "1" << std::endl;
  }
  newfile << "0" << std::endl;
  newfile.close();
  std::cout << "Motion plan saved to " << newfilename << std::endl;
}

void saveMotionPlanToCSV(const std::vector<Eigen::VectorXd> &motion_plan,
                         const std::string &filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return;
  }

  for (const auto &joint_values : motion_plan) {
    for (int j = 0; j < joint_values.size(); ++j) {
      file << joint_values[j];
      if (j < joint_values.size() - 1) {
        file << ",";
      }
    }
    file << std::endl;
  }

  file.close();
  std::cout << "Motion plan saved to " << filename << std::endl;
}

void saveGripperConditionToCSV(
    const std::vector<double> &guiding_pose_gripper_cond,
    const std::string &filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return;
  }

  // Write all elements on one line, separated by commas
  for (size_t i = 0; i < guiding_pose_gripper_cond.size(); ++i) {
    file << guiding_pose_gripper_cond[i];
    if (i < guiding_pose_gripper_cond.size() - 1) {
      file << ",";
    }
  }
  file << std::endl;

  file.close();
  std::cout << "Motion plan (gripper condition) saved to " << filename
            << std::endl;
}

std::vector<double> readGripperCondition(const std::string &filename) {
  std::vector<double> values;
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "reading gripper cond: " << filename << std::endl;
    return values;
  }
  std::string line;
  std::cerr << line << std::endl;
  if (std::getline(file, line)) {
  }
  while (std::getline(file, line)) {
    if (line.empty())
      continue;

    while (!line.empty() &&
           (line.back() == '\r' || line.back() == '\n' || line.back() == ' ')) {
      line.pop_back();
    }
    while (!line.empty() && line.front() == ' ') {
      line.erase(0, 1);
    }

    // 2. Skip empty lines
    if (line.empty())
      continue;

    // 3. Skip meta labels
    if (line.find(":") != std::string::npos ||
        line.find("gst") != std::string::npos) {
      continue;
    }
    try {
      double value = std::stod(line);
      values.push_back(value);
    } catch (const std::invalid_argument &e) {
      std::cerr << "Couldn't convert line to number: " << line << std::endl;
    } catch (const std::out_of_range &e) {
      std::cerr << "number out of range: " << line << std::endl;
    }
  }
  file.close();
  return values;
}

void makeSimulationGripperCSV(const std::string &newfilename,
                              const std::string &filename, const int pre_joint,
                              const int drop_joint) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return;
  }
  std::string header;
  std::getline(file, header);
  std::string line;
  int rowCount = 0;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    bool isOnlyWhitespace =
        std::all_of(line.begin(), line.end(),
                    [](unsigned char c) { return std::isspace(c); });

    if (!line.empty() && !isOnlyWhitespace) {
      rowCount++;
    }
  }
  file.close();
  std::cerr << rowCount << std::endl;

  std::ofstream newfile(newfilename);
  if (!newfile.is_open()) {
    std::cerr << "Error: Could not open file " << newfilename << std::endl;
    return;
  }
  newfile << "0" << ',';
  for (int j = 0; j < rowCount - 1; ++j) {
    std::cout << j << " " << pre_joint << std::endl;
    if (j<pre_joint | j> drop_joint) {
      newfile << "0" << ',';
      continue;
    }
    newfile << "1" << ',';
  }
  newfile << "0" << std::endl;
  newfile.close();
  std::cout << "Motion plan saved to " << newfilename << std::endl;
}

int main() {
  Eigen::IOFormat CleanFmt(Eigen::FullPrecision, 0, "\t", "\n");

  makeDummyGripperCSV(
      std::string(KINLIB_RESOURCES_DIR) +
          "Demonstrations/aria_project/dummy_gripper.csv",
      std::string(KINLIB_RESOURCES_DIR) +
          "Demonstrations/aria_project/world_object_formatted_smoothed.csv");

  kinlib::Manipulator panda_manipulator;

  // moving into the joints
  // CHANGE FOR DIFF ROBOT
  // these are specifically for baxter but can be changed
  std::array<std::string, 7> joint_names{
      "pandaJoint1", "pandaJoint2", "pandaJoint3", "pandaJoint4",
      "pandaJoint5", "pandaJoint6", "pandaJoint7"};

  Eigen::MatrixXd joint_axes = loadCSV<Eigen::MatrixXd>(
      std::string(KINLIB_RESOURCES_DIR) + "panda_joint_axes.csv");
  Eigen::MatrixXd joint_q = loadCSV<Eigen::MatrixXd>(
      std::string(KINLIB_RESOURCES_DIR) + "panda_joint_q.csv");
  Eigen::MatrixXd joint_limits = loadCSV<Eigen::MatrixXd>(
      std::string(KINLIB_RESOURCES_DIR) + "panda_joint_limit.csv");
  Eigen::MatrixXd gst_0 = loadCSV<Eigen::MatrixXd>(
      std::string(KINLIB_RESOURCES_DIR) + "panda_gst0.csv");
  // CHANGE FOR DIFF ROBOT

  // CHANGE ONCE NOT DUMMY
  std::vector<double> gripper_condition =
      readGripperCondition(std::string(KINLIB_RESOURCES_DIR) +
                           "Demonstrations/aria_project/dummy_gripper.csv");

  std::cout << "gst0 :\n" << gripper_condition.size() << "\n\n";
  // std::cout << "check" << std::endl;

  for (int i = 0; i < 7; i++) {
    if (i >= joint_axes.cols() || i >= joint_q.cols()) {
      std::cerr << "CRITICAL: Loop index i=" << i
                << " exceeds available matrix columns (" << joint_axes.cols()
                << ")!" << std::endl;
      break;
    }
    // std::cout << joint_axes.cols() << std::endl;
    // std::cout << joint_q << std::endl;

    Eigen::Vector4d jnt_axis;
    jnt_axis.head<3>() = joint_axes.block<3, 1>(0, i);
    jnt_axis(3) = 0;

    Eigen::Vector4d jnt_q;
    jnt_q.head<3>() = joint_q.block<3, 1>(0, i);
    jnt_q(3) = 0;

    kinlib::JointLimits jnt_limits;
    jnt_limits.lower_limit_ = joint_limits(i, 0);
    jnt_limits.upper_limit_ = joint_limits(i, 1);

    panda_manipulator.addJoint(kinlib::JointType::Revolute, joint_names[i],
                               jnt_axis, jnt_q, jnt_limits, gst_0);
  }
  //   std::cout << "check" << std::endl;

  kinlib::KinematicsSolver kin_solver(panda_manipulator);

  Eigen::MatrixXd recorded_demo = loadCSV<Eigen::MatrixXd>(
      std::string(KINLIB_RESOURCES_DIR) +
      "Demonstrations/aria_project/world_object_formatted_smoothed.csv");

  Eigen::MatrixXd object_poses = loadCSV<Eigen::MatrixXd>(
      std::string(KINLIB_RESOURCES_DIR) +
      "Demonstrations/aria_project/object_poses_pipeline_smoothed.csv");

  std::vector<Eigen::Matrix4d> recorded_ee_traj;
  std::vector<Eigen::Matrix4d> obj_poses;
  Eigen::Matrix4d end_effector_offset;
  // would this change based on file
  end_effector_offset << 0, -1, 0, 0, 0, 0, 1, -0.1034, -1, 0, 0, 0, 0, 0, 0, 1;
  for (int i = 0; i < recorded_demo.rows() / 4; i++) {
    Eigen::Matrix4d g = recorded_demo.block<4, 4>((i * 4), 0);
    g = g * end_effector_offset;
    recorded_ee_traj.push_back(g);
  }
  // std::cout << "demo" << std::endl;

  for (int i = 0; i < object_poses.rows() / 4; i++) {
    Eigen::Matrix4d g = object_poses.block<4, 4>((i * 4), 0);
    obj_poses.push_back(g);
  }

  std::cout << "ee_pos before filtered is :\n"
            << recorded_ee_traj.size() << std::endl;

  std::vector<Eigen::Matrix4d> filtered_ee_traj;
  std::vector<double> filtered_gripperCond;
  std::vector<unsigned int> gripper_change_index;
  kinlib::filterSE3Sequence(recorded_ee_traj, filtered_ee_traj,
                            gripper_condition, filtered_gripperCond,
                            gripper_change_index);

  for (int i = 0; i < gripper_change_index.size(); i++) {
    std::cout << "\ngripper_change_index are " << gripper_change_index[i]
              << '\n';
  }

  std::vector<double> guiding_pose_gripper_cond;
  std::cout << "ee_pos after filtered is :\n"
            << filtered_ee_traj.size() << std::endl;
  kinlib::Demonstration demo = kinlib::saveDemonstration(
      filtered_ee_traj, obj_poses, filtered_gripperCond,
      guiding_pose_gripper_cond, gripper_change_index, 0.3);

  // Guiding poses

  // this only gives us one set of guiding poses here
  // for the new one
  // why would this be
  // fig out

  std::cout << demo.guiding_poses.size() << std::endl;
  //   for(int i = 0; i < demo.guiding_poses.size(); i++)
  //   {
  //     std::cout << "\nGuiding poses associated with object " << i+1 << '\n';
  //     for(int j = 0; j < demo.guiding_poses[i].size(); j++)
  //     {
  //       std::cout << demo.guiding_poses[i][j] << '\n';
  //     }
  //   }
  std::cout << "check" << std::endl;

  kinlib::TaskInstance new_task_instance;

  new_task_instance.object_poses = demo.task_instance.object_poses;

  new_task_instance.object_poses[0](0, 3) = 0.5;
  new_task_instance.object_poses[0](1, 3) = -0.1;
  new_task_instance.object_poses[0](2, 3) = 0.35;
  new_task_instance.object_poses[1](0, 3) = 0.5;
  new_task_instance.object_poses[1](1, 3) = -0.3;
  new_task_instance.object_poses[1](2, 3) = 0.35;

  std::cout << new_task_instance.object_poses[1] << '\n';

  std::vector<Eigen::Matrix4d> new_motion_plan;
  kinlib::ErrorCodes res =
      kinlib::UserGuidedMotionPlanner::planMotionForNewTaskInstance(
          demo, new_task_instance, new_motion_plan);
  std::cout << "boom" << std::endl;
  saveGuidingPosesRaw("guiding_poses.csv", new_motion_plan);
  makeDummyGripperCSV("new_gripper.csv", "guiding_poses.csv");

  std::vector<double> motion_plans_gripper_cond;
  Eigen::VectorXd init_jnt_val(7);
  init_jnt_val << 0., -0.7854, 0., -2.3562, 0., 1.5708, 0.7854;
  Eigen::Matrix4d init_ee_g;
  std::vector<Eigen::VectorXd> all_motion_plans;
  kin_solver.getFK(init_jnt_val, init_ee_g);
  Eigen::Matrix4d end_pose;
  end_pose = init_ee_g;
  //   new_task_instance.object_poses[0](0,3) += 0.5;
  //   new_task_instance.object_poses[0](1,3) -= -0.1;
  end_pose(2, 3) += 0.051;

  new_motion_plan.push_back(end_pose);
  saveGuidingPosesRaw("guiding_poses_back_to_start.csv", new_motion_plan);
  makeDummyGripperCSV("new_gripper_back_to_start.csv",
                      "guiding_poses_back_to_start.csv");

  int steps = 0;
  int pre_gripper_joints;
  int end_block_joints;
  for (const auto &goal_ee_g : new_motion_plan) {
    steps += 1;
    // kin_solver.getFK(init_jnt_val, init_ee_g);
    kinlib::MotionPlanResult plan_info;
    std::vector<Eigen::VectorXd> motion_plan_result;
    kinlib::ErrorCodes plan_res = kin_solver.getMotionPlan(
        init_jnt_val, init_ee_g, goal_ee_g, motion_plan_result, plan_info);

    if (plan_res != kinlib::ErrorCodes::OPERATION_SUCCESS) {
      std::cout << "###########################\nPlan "
                   "failed!\n###########################\n";
    }
    // kinlib::ErrorCodes plan_res =
    // kin_solver.getMotionPlanWithNSP(panda,init_jnt_val, init_ee_g, goal_ee_g,
    // motion_plan_result, plan_info,outer_threshold, inner_threshold);
    std::cout << "init pos: " << init_ee_g << std::endl;
    std::cout << "goal pos: " << goal_ee_g << std::endl;
    all_motion_plans.insert(all_motion_plans.end(), motion_plan_result.begin(),
                            motion_plan_result.end());
    std::cout << "is this the seg err" << std::endl;
    if (steps == 1) {
      pre_gripper_joints = motion_plan_result.size();
      std::cout << pre_gripper_joints << std::endl;
    }
    if (steps < new_motion_plan.size()) {
      end_block_joints += motion_plan_result.size();
    }
    init_jnt_val = motion_plan_result.back();

    std::cout << "number of steps: " << steps << std::endl;
    std::cout << init_jnt_val << std::endl;
  }

  saveMotionPlanToCSV(all_motion_plans, "project_aria.csv");
  makeSimulationGripperCSV("simulation_gripper.csv", "project_aria.csv",
                           pre_gripper_joints - 1, end_block_joints - 1);

  // saveGripperConditionToCSV(motion_plans_gripper_cond,
  // "build_wall_gripper_condition.csv");

  // and then from THIS we do rigid transform
  // ***INSERT RIGID TRANSFORM HERE*****
  // WHICH IS BASED ON SPECIF ROBOT WE R USING
  // && i will do this later
}
