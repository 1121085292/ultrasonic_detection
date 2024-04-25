#include "parking_spot_detection.h"

void ParkingSpotDetection::ParkingSpotSearch(
    const Point2D& point, const Pose &pose){
  std::vector<Point2D> points;
  points.emplace_back(point);
  // 当障碍物点集≥10时，才进行特征线段拟合
  if(points.size() < LineFitParams().min_fit_num){
    return;
  }
  // 特征线段拟合
  line_segment_ptr_ = std::make_shared<LineSegment>();
  std::vector<LineSegment> fit_lines = line_segment_ptr_->FitLineSegments(points);
  // 1.潜在车位搜索
  std::map<int, std::vector<LineSegment>> line_segments;
  FindPotentialParkingSpots(fit_lines, pose, line_segments);
  // 2.对存在潜在车位的线段对再判断
  for(const auto& pair : line_segments){
    if(!ReEvaluateParallelParkingSpot(pair, pose)) continue;

    // 3.车位约束条件判断
    if(!CheckParkingSpotConstraints(line_segment)) continue;

    // 4.泊车位，车位位姿计算
    double angle = CalculateParkingSpotAngle();
  }
}

void ParkingSpotDetection::FindPotentialParkingSpots(
    vector<LineSegment> &fit_lines, const Pose &pose,
    std::map<int, std::vector<LineSegment>>& line_segments){
  for (size_t i = 0; i < fit_lines.size() - 1; ++i) {
    fit_lines_map_[i] = fit_lines[i];
    LineSegment& line1 = fit_lines[i];
    LineSegment& line2 = fit_lines[i + 1];

    // 判断特征线段长度是否满足阈值
    if (line1.Length() < LineFitParams().min_fit_distance ||
        line2.Length() < LineFitParams().min_fit_distance) continue;

    // 将线段投影到车辆行驶方向
    LineSegment line1_proj = line1.ProjectOnto(pose.heading());
    LineSegment line2_proj = line2.ProjectOnto(pose.heading());

    // 计算投影后的线段近端点距离
    double near_dist = (line1_proj.GetEnd() - line2_proj.GetStart()).Length();
    double far_dist = (line1_proj.GetStart() - line2_proj.GetEnd()).Length();
    
    double proj_dist = std::min(near_dist, far_dist);

    // 判断近端点距离是否大于车位宽度阈值
    // 此处无法判断车位类型，以宽度较小的垂直车位进行判断
    std::vector<LineSegment> lines;
    if (proj_dist > ParkingSpotParams().spot_width_threshold) {
      lines.emplace_back(line1);
      lines.emplace_back(line2);
      line_segments[i] = lines;
    }
  }
}

// 对存在潜在车位的线段对再判断
bool ParkingSpotDetection::ReEvaluateParallelParkingSpot(
    std::map<int, std::vector<LineSegment>> &line_segment_pair,
    const Pose &pose){
  int key = line_segment_pair.first;
  LineSegment& line1 = line_segment_pair.second.front();
  LineSegment& line2 = line_segment_pair.second.back();

  // 生长后线段的投影累计长度
  double proj_length1 = GrowLineSegment(line1, key, true, pose);
  double proj_length2 = GrowLineSegment(line1, key, false, pose);

  // 判断投影长度是否大于长度阈值
  if(proj_length1 > ParkingSpotParams().line_length_threshold &&
     proj_length2 > ParkingSpotParams().line_length_threshold){
    parking_space_type_ = ParkingSpaceType::PARALLEL_PARKING;
    return true;
  // 判断投影长度是否大于宽度阈值
  } else if(proj_length1 > ParkingSpotParams().line_width_threshold &&
            proj_length2 > ParkingSpotParams().line_width_threshold){
    parking_space_type_ = ParkingSpaceType::VERTICAL_PLOT;
    return true;
  }

  AINFO << "ReEvaluate Parallel Parking Spot Failed.";
  return false;
}

bool ParkingSpotDetection::CheckParkingSpotConstraints(
  const pair<LineSegment, LineSegment> &parkingSpot){

  return false;
}

double ParkingSpotDetection::CalculateParkingSpotAngle(
    const pair<LineSegment, LineSegment> &parking_spot,
    const vector<LineSegment> &curbs,
    const vector<LineSegment> &obstacles,
    double vehicleHeading){


  return 0.0;
}

double ParkingSpotDetection::GrowLineSegment(
    LineSegment &line,int key,
    bool flag, const Pose& pose){
  // 投影线段长度
  double project_length = line.ProjectLength(pose.heading());
  if(flag){
    // line1向后生长
    while(1){
      if(key == 0) return project_length;
      // 相邻线段
      auto pre_line = fit_lines_map_[--key];
      // 判断两条拟合线段距离是否满足阈值
      if((line.GetStart() - pre_line.GetEnd()).Length() > ParkingSpotParams().line_dist){
        return project_length;
      // 判断两条拟合线段夹角是否满足阈值
      } else if(AngleBetweenLines(line, pre_line) > ParkingSpotParams().line_angle){
        return project_length;
      } else {
        // 生长
        project_length += pre_line.ProjectLength(pose.heading());
        // 更新
        line = pre_line;
      }
    }
  } else {
    // line2 向前生长
    while(1){
      if(key == static_cast<int>(fit_lines_map_.size())) return project_length;
      // 相邻线段
      auto next_line = fit_lines_map_[++key];
      // 判断两条拟合线段距离是否满足阈值
      if((next_line.GetStart() - line.GetEnd()).Length() > ParkingSpotParams().line_dist){
        return project_length;
      // 判断两条拟合线段夹角是否满足阈值
      } else if(AngleBetweenLines(next_line, line) > ParkingSpotParams().line_angle){
        return project_length;
      } else {
        // 生长
        project_length += next_line.ProjectLength(pose.heading());
        // 更新
        line = next_line;
      }
    }
  }
  // return project_length;
}

double ParkingSpotDetection::AngleBetweenLines(
    const LineSegment &line1, const LineSegment &line2){
  auto dir1 = line1.GetDirection();
  auto dir2 = line2.GetDirection();

  double dot = dir1.dot(dir2);
  // 弧长
  return fabs(acos( dot / (line1.Length() * line2.Length())));
}