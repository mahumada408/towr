/*!
 * \file   sample_qp_caller.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 5, 2016
 * \brief  sends out a sample current state and footholds to the QP server
 */

#include <xpp/ros/ros_helpers.h>
#include <xpp_msgs/RequiredInfoQp.h>          // send
#include <xpp_msgs/OptimizedParametersQp.h>   // receive

#include <xpp/ros/marker_array_builder.h>
#include <xpp/opt/com_spline6.h>
#include <xpp/opt/motion_structure.h>

typedef xpp_msgs::RequiredInfoQp ReqInfoMsg;
typedef xpp_msgs::OptimizedParametersQp OptimizedParametersMsg;
using MotionStructure = xpp::opt::MotionStructure;

xpp::opt::ComSpline6::VecPolynomials splines;
std::vector<xpp::hyq::Foothold> footholds;
void OptParamsCallback(const OptimizedParametersMsg& msg)
{
  splines = xpp::ros::RosHelpers::RosToXpp(msg.splines);
}

int main(int argc, char **argv)
{
  using namespace xpp::ros;
  if (argc==1) ROS_FATAL("Please specify current x- velocity & acceleration as parameters");

  ros::init(argc, argv, "sample_qp_caller");
  ros::NodeHandle n;
  ros::Subscriber opt_params_sub = n.subscribe("optimized_parameters_qp", 1, OptParamsCallback);
  ros::Publisher current_info_pub = n.advertise<ReqInfoMsg>("required_info_qp", 1);\

  // give publisher and subscriber time to register with master
  ros::Rate poll_rate(100);
  while(opt_params_sub.getNumPublishers() == 0 || current_info_pub.getNumSubscribers() == 0) {
    poll_rate.sleep(); // so node has time to connect
  }


  ReqInfoMsg msg;
  msg.curr_state.pos.x = 0.0;
  msg.curr_state.vel.x = atof(argv[1]); // figure out why this fails
  msg.curr_state.vel.y = atof(argv[1]); // figure out why this fails
  msg.curr_state.acc.x = atof(argv[2]); // this is a constraint
  msg.curr_state.acc.y = atof(argv[2]); // this is a constraint
//  msg.curr_state.acc.y = 1.0;

  using namespace xpp::hyq;
  xpp::hyq::LegDataMap<xpp::hyq::Foothold> start_stance;
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold( 0.35+msg.curr_state.pos.x,  0.3, 0.0, LF)));
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold( 0.35+msg.curr_state.pos.x, -0.3, 0.0, RF)));
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold(-0.35+msg.curr_state.pos.x,  0.3, 0.0, LH)));
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold(-0.35+msg.curr_state.pos.x, -0.3, 0.0, RH)));

  double step_length = 0.25;
  msg.steps.push_back(RosHelpers::XppToRos(Foothold(-0.35 + step_length,  0.3, 0.0, LH)));
  msg.steps.push_back(RosHelpers::XppToRos(Foothold( 0.35 + step_length,  0.3, 0.0, LF)));
  msg.steps.push_back(RosHelpers::XppToRos(Foothold(-0.35 + step_length, -0.3, 0.0, RH)));
  msg.steps.push_back(RosHelpers::XppToRos(Foothold( 0.35 + step_length, -0.3, 0.0, RF)));
  // this 5th step is neccessary, so a goal position of x+=0.25 can be reached,
  // as the last step's (RF) support polygon otherwise can't be used
  msg.steps.push_back(RosHelpers::XppToRos(Foothold(-0.35 + 2*step_length, 0.3, 0.0, LH)));

  msg.start_with_com_shift = true;

  current_info_pub.publish(msg);


  xpp::ros::MarkerArrayBuilder marker_builder;
  double walking_height = RosHelpers::GetDoubleFromServer("/xpp/robot_height");
  ros::Publisher ros_publisher_ = n.advertise<visualization_msgs::MarkerArray>("optimization_variables", 1);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    visualization_msgs::MarkerArray marker_msg;
    marker_builder.AddFootholds(marker_msg, RosHelpers::RosToXpp(msg.steps), "footholds", visualization_msgs::Marker::CUBE, 1.0);
//    marker_builder.AddCogTrajectory(marker_msg, splines, RosHelpers::RosToXpp(msg.steps), "cog", 1.0);
//    marker_builder.AddZmpTrajectory(marker_msg, splines, walking_height, RosHelpers::RosToXpp(msg.steps), "zmp_4ls", 0.7);

    ros_publisher_.publish(marker_msg);
    loop_rate.sleep();
  }
}


