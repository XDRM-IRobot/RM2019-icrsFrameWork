#include "kalman_node.h"
std::vector<float> Pre_vels_(9);
std::vector<float> Pre_vels_copy(9);
using namespace cv;

bool comp(const float &a, const float &b)
{
    return abs(a) > abs(b);
}

 float kalman_node::PreFilter(float Pre_vel)
{

  Pre_vels_[0] = Pre_vels_[1];
  Pre_vels_[1] = Pre_vels_[2];
  Pre_vels_[2] = Pre_vels_[3];
  Pre_vels_[3] = Pre_vels_[4];
  Pre_vels_[4] = Pre_vels_[5];
  Pre_vels_[5] = Pre_vels_[6];
  Pre_vels_[6] = Pre_vels_[7];
  Pre_vels_[7] = Pre_vels_[8];
  Pre_vels_[8] = Pre_vel;

  Pre_vels_copy = Pre_vels_;
  sort(Pre_vels_copy.begin(), Pre_vels_copy.end(), comp);
  // ROS_ERROR("%f", Pre_vels_copy[0]);
  // ROS_ERROR("%f", Pre_vels_copy[1]);
  // ROS_ERROR("%f", Pre_vels_copy[2]);
  // ROS_ERROR("%f", Pre_vels_copy[3]);
  // ROS_ERROR("%f", Pre_vels_copy[4]);
  // ROS_ERROR("%f", Pre_vels_copy[5]);
  // ROS_ERROR("%f", Pre_vels_copy[6]);
  // ROS_ERROR("%f", Pre_vels_copy[7]);
  // ROS_ERROR("%f", Pre_vels_copy[8]);
  // ROS_ERROR("--------------------------------------------------------------------");
  // ROS_ERROR("%f", Pre_vels_[0]);
  // ROS_ERROR("%f", Pre_vels_[1]);
  // ROS_ERROR("%f", Pre_vels_[2]);
  // ROS_ERROR("%f", Pre_vels_[3]);
  // ROS_ERROR("%f", Pre_vels_[4]);
  // ROS_ERROR("%f", Pre_vels_[5]);
  // ROS_ERROR("%f", Pre_vels_[6]);
  // ROS_ERROR("%f", Pre_vels_[7]);
  // ROS_ERROR("%f", Pre_vels_[8]);
  // ROS_ERROR("=====================================================================");
  return Pre_vels_copy[4];
}

cv::RotatedRect kalman_node::KF(cv::RotatedRect  armor_rect)
{
  if(loop_cnt < 1) 
  hisRect = armor_rect;
  loop_cnt++;

  KalmanFilter KF(4, 2);
  Mat state(4, 1, CV_32F); 
  Mat processNoise(4, 1, CV_32F);                     
  Mat measurement = Mat::zeros(2, 1, CV_32F);         
  KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,
                                              0, 1, 0, 1,
                                              0, 0, 1, 0,
                                              0, 0, 0, 1 ); 
  setIdentity(KF.measurementMatrix); 
  setIdentity(KF.processNoiseCov, Scalar::all(1e-5));     
  setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1)); 
  setIdentity(KF.errorCovPost, Scalar::all(1));           

  //初始化运动状态
  vel_x = -(hisRect.center.x - armor_rect.center.x)*2.4 + kalman_yaw_rate;
  vel_x = vel_x*2.2;
  // vel_y = -(hisRect.center.y - armor_rect.center.y)*0;

  state.at<float>(0) = armor_rect.center.x;   //x
  state.at<float>(1) = armor_rect.center.y;   //y
  state.at<float>(2) = hisRect.center.x - armor_rect.center.x;   //vx;
  state.at<float>(3) = hisRect.center.y - armor_rect.center.y;   //vy;
    
  VelFliter(vel_x);

  // ROS_ERROR("vel_x: %f", vel_x);
  // ROS_ERROR("max_vel: %f", max_vel_x);
  // ROS_ERROR("op_max_vel: %f", op_max_vel_x);   
  // ROS_ERROR("min_vel: %f", min_vel_x) ;
  // ROS_ERROR("op_min_vel: %f", op_min_vel_x);        
    
    KF.statePost.at<float>(0) = armor_rect.center.x;
    KF.statePost.at<float>(1) = armor_rect.center.y;
    KF.statePost.at<float>(2) = vel_x ;
    KF.statePost.at<float>(3) = vel_y ;

    old_his_vel_x = his_vel_x;
    his_vel_x = vel_x;

    Point statePt = Point(state.at<float>(0), state.at<float>(1));                                                                          //这边这个状态向量 1.角度 2.速度（估计是）
    Mat prediction = KF.predict();
    double px = prediction.at<float>(0);
    double py = prediction.at<float>(1);
    Point predictPt = (Point2f(px,py)); //change x for miao zhun

    randn( measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));

    // generate measurement
    measurement += KF.measurementMatrix*state;
    double mx = measurement.at<float>(0);
    double my = measurement.at<float>(1);
    Point measPt = Point(mx,my);
    if(theRNG().uniform(0,4) != 0)
        KF.correct(measurement);
    randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
    state = KF.transitionMatrix*state + processNoise;

    PreRect = RotatedRect(predictPt,armor_rect.size,armor_rect.angle);
    hisRect = armor_rect;
    his_vx  = -(hisRect.center.x - armor_rect.center.x)*1.8;
    hisprerect = PreRect;

    return PreRect; //output
}

void kalman_node::VelFliter(float &vel_x)
{
  //feng zhi bao chi shi jian
  if(loop_cnt > 4)    
  {
    loop_cnt = 0;
    max_vel_x = 0;
    min_vel_x = 100000;
    op_max_vel_x = 0;
    op_min_vel_x = -100000;
    his_max_vel_x = 0;
    his_min_vel_x = 0;
    old_his_vel_x = 0;
  }
  //zhong zhi
  int temp_vel = PreFilter(vel_x);
  //feng zhi bao chi
  if(temp_vel > 0 && (temp_vel - his_vel_x) > -10) //jia su du a>0 su du v>0
  {
    if(temp_vel > max_vel_x )
    {
      max_vel_x = temp_vel;
    }
  }
  if(temp_vel > 0 && (temp_vel - his_vel_x) < 10) //jia su du a<0 su du v>0
  {
    if(temp_vel < min_vel_x)
    {
      min_vel_x = temp_vel;
    }
  }
  if(temp_vel < 0 && (temp_vel - his_vel_x) < 10) //jia su du a>0 su du v<0
  {
    if(temp_vel < op_max_vel_x )
    {
      op_max_vel_x = temp_vel;
    }
  }
  if(temp_vel < 0 && (temp_vel - his_vel_x) > -10) //jia su du a<0 su du v<0
  {
    if(temp_vel > op_min_vel_x)
    {
      op_min_vel_x = temp_vel;
    }
  } 
  if ( vel_x > 2 && (vel_x - his_vel_x) > -2 /*vel_x > 2*/)  //a>0, v>0
  {
    vel_x = max_vel_x;
    his_max_vel_x = vel_x;
  }
  else if ( vel_x > 2 && (vel_x - his_vel_x) < 2)           //a<0, v>0
  {
    vel_x = min_vel_x;
    his_min_vel_x = vel_x;
  }
  else if (vel_x < -2 && (vel_x - his_vel_x) < 2 /*vel_x < -2*/)  //a>0, v<0
  {
    vel_x = op_max_vel_x;
    his_max_vel_x = vel_x;
  }
  else if (vel_x < -2 && (vel_x - his_vel_x) > -2)             //a<0, v<0
  {
    vel_x = op_min_vel_x;
    his_min_vel_x = vel_x;
  }
  else
  {
    // vel_x = his_min_vel_x;
    vel_x = his_max_vel_x;
  }
  //sha che qing kuang
  if((vel_x - old_his_vel_x < -10) && vel_x > 0)
  {
    vel_x = 0;
    ROS_ERROR("+ sha che");
    // breaking_cnt++;
  }      
  else if ((vel_x - old_his_vel_x > 10) && vel_x < 0)
  {
    vel_x = 0;
    ROS_ERROR("- sha che");
    
  }
  //fan xiang
  if (his_vel_x != 0 && ((his_vel_x > 0 && vel_x < 0) || (his_vel_x < 0 &&  vel_x > 0) ) ) 
  {
      vel_x = his_vel_x;
      // vel_x = 0;
  }
  //xian fu
  if ( std::abs(vel_x)  >120) 
  {
      // vel_x = 0;
      vel_x = his_vel_x;
  }
  // ROS_ERROR("vel_x: %f", vel_x);
  // ROS_ERROR("max_vel: %f", max_vel_x);
  // ROS_ERROR("op_max_vel: %f", op_max_vel_x);   
  // ROS_ERROR("min_vel: %f", min_vel_x) ;
  // ROS_ERROR("op_min_vel: %f", op_min_vel_x);
}


 