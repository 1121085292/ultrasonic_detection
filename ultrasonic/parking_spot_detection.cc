#include "parking_spot_detection.h"

void ParkingSpotDetection::ParkingSpotSearch(
    const Point2D& point, const std::shared_ptr<Pose> &pose,
    const LineFitParams& line_fit_params,
    const ParkingSpotParams& parking_spot_params,
    const CurbParams& curb_params,
    ParkingSpot& parking_spot){
  points_.emplace_back(point);
  // 当障碍物点集≥10时，才进行特征线段拟合
  if(static_cast<int>(points_.size()) < line_fit_params.min_fit_num){
    return;
  }
  // 特征线段拟合
  line_segment_ptr_ = std::make_shared<LineSegment>();
  std::vector<LineSegment> fit_lines = line_segment_ptr_->FitLineSegments(points_, line_fit_params);
  // 1.潜在车位搜索
  std::map<int, std::vector<LineSegment>> line_segments;
  if(!FindPotentialParkingSpots(fit_lines, pose, parking_spot_params, curb_params, line_segments)){
    return;
  }
  // 2.对存在潜在车位的线段对再判断
  for(const auto& pair : line_segments){
    // 满足车位再判断的线段对
    if(!ReEvaluateParallelParkingSpot(pair, parking_spot_params, pose)) continue;

    // 3.车位约束条件判断
    if(!CheckParkingSpotConstraints(pose, parking_spot_params)) continue;

    // 4.车位位姿计算
    double angle = CalculateParkingSpotAngle(pose);
    // 5.泊车位角点计算
    CalculateParkingVertices(angle, parking_spot);
  }
}

bool ParkingSpotDetection::FindPotentialParkingSpots(
    const std::vector<LineSegment> &fit_lines,
    const std::shared_ptr<Pose> &pose,
    const ParkingSpotParams& parking_spot_params,
    const CurbParams& curb_params,
    std::map<int, std::vector<LineSegment>>& line_segments){
  if(fit_lines.empty()) return false;
  // 筛选出路牙线段和障碍物线段
  for (size_t i = 0; i < fit_lines.size() - 1; ++i) {
    const LineSegment& line = fit_lines[i];
    // 判断是否为路牙线段
    if(IsCurbLine(line, curb_params, pose)){
      curb_lines_.emplace_back(line);
    } else {
      fit_lines_.emplace_back(line);
    }
  }
  for(size_t i = 0; i < fit_lines.size() - 1; ++i){
    LineSegment line1 = fit_lines[i];
    const LineSegment& line2 = fit_lines[i + 1];
    // 判断特征线段长度是否满足阈值
    if (line1.Length() < parking_spot_params.min_line_length ||
        line2.Length() < parking_spot_params.min_line_length) continue;
    // 线段近端点组成新的线段
    LineSegment line(line1.GetEnd(), line2.GetStart());
    // 计算投影长度
    double proj_dist = line.ProjectLength(pose->heading());

    // 判断近端点距离是否大于车位宽度阈值
    std::vector<LineSegment> lines;
    if (proj_dist > ParkingSpotParams().spot_width_threshold) {
      lines.emplace_back(line1);
      lines.emplace_back(line2);
      line_segments[i] = lines;
      // 无路牙时不成立
      // if(curb_lines_map_.find(i + 1) != curb_lines_map_.end()){
      //   parking_spot_type_ = ParkingSpotType::PARALLEL_PARKING;
      // } else {
      //   parking_spot_type_ = ParkingSpotType::VERTICAL_PLOT;
      // }
      return true;
    }
  }
  return false;
}

// 对存在潜在车位的线段对再判断
bool ParkingSpotDetection::ReEvaluateParallelParkingSpot(
    const std::pair<int, std::vector<LineSegment>> &line_segment_pair,
    const ParkingSpotParams& parking_spot_params,
    const std::shared_ptr<Pose> &pose){
  // 通过key找前后线段
  int key = line_segment_pair.first;
  LineSegment line1 = line_segment_pair.second.front();
  LineSegment line2 = line_segment_pair.second.back();

  // 生长后线段的投影累计长度
  double proj_length1 = GrowLineSegment(line1, key, true, parking_spot_params, pose);
  double proj_length2 = GrowLineSegment(line2, key, false, parking_spot_params, pose);

  // 判断投影长度是否大于长度阈值
  if(proj_length1 > ParkingSpotParams().line_length_threshold &&
     proj_length2 > ParkingSpotParams().line_length_threshold){
    parking_spot_type_ = ParkingSpotType::PARALLEL_PARKING;
    parking_spot_pair_.emplace_back(line1);
    parking_spot_pair_.emplace_back(line2);
    return true;
  // 判断投影长度是否大于宽度阈值
  } else if(proj_length1 > ParkingSpotParams().line_width_threshold &&
            proj_length2 > ParkingSpotParams().line_width_threshold){
    parking_spot_type_ = ParkingSpotType::VERTICAL_PLOT;
    parking_spot_pair_.emplace_back(line1);
    parking_spot_pair_.emplace_back(line2);
    return true;
  }

  AINFO << "ReEvaluate Parallel Parking Spot Failed.";
  return false;
}

bool ParkingSpotDetection::CheckParkingSpotConstraints(
    const std::shared_ptr<Pose>& pose,
    const ParkingSpotParams& parking_spot_params){
  if(parking_spot_pair_.empty()){
    return false;
  }
  LineSegment line1 = parking_spot_pair_[0];
  LineSegment line2 = parking_spot_pair_[1];

  // 车位深度
  double depth = 0.0;
  // 车位长度
  double length = 0.0;
  // 横向偏移
  double offset = 0.0;
  // 偏移角度
  double angle = 0.0;
  if(curb_lines_.empty()){
    depth = 6000.0;
  } else {
    double dist1 = line1.DistanceToLine(line1.GetEnd(), curb_lines_[0]);
    double dist2 = line2.DistanceToLine(line2.GetStart(), curb_lines_[0]);
    // 车位深度
    depth = std::min(dist1, dist2);
    // 车位长度
    // 线段近端点组成新的线段
    LineSegment line(line1.GetEnd(), line2.GetStart());
    length = line.ProjectLength(pose->heading());
    // 横向偏移
    offset = std::max(offset, LineSegment().DistanceToLine(line1.GetStart(), line2));
    offset = std::max(offset, LineSegment().DistanceToLine(line1.GetEnd(), line2));
    offset = std::max(offset, LineSegment().DistanceToLine(line2.GetStart(), line1));
    offset = std::max(offset, LineSegment().DistanceToLine(line2.GetStart(), line1));
    // 偏移角度
    angle = line1.AngleBetweenLines(line2);
  }
  if(offset > parking_spot_params.offset_threshold && angle > parking_spot_params.angle_threshold * M_PI / 180){
    return false;
  }
  if(parking_spot_type_ == ParkingSpotType::PARALLEL_PARKING){
    if(depth < ParkingSpotParams().spot_width_threshold &&
       length < ParkingSpotParams().spot_length_threshold){
      return true;
    }
  } else if(parking_spot_type_ == ParkingSpotType::VERTICAL_PLOT){
    if(depth < ParkingSpotParams().spot_length_threshold &&
       length < ParkingSpotParams().spot_width_threshold){
      return true;
    }
  }
  return false;
}

double ParkingSpotDetection::CalculateParkingSpotAngle(const std::shared_ptr<Pose>& pose){
  if(!curb_lines_.empty()){
    // 有路牙线段用路牙线段计算
    Point2D dir = curb_lines_[0].GetDirection();
    return atan2(dir.y, dir.x);
  } else {
    // 用特征线段
    Point2D dir1 = parking_spot_pair_[0].GetDirection();
    Point2D dir2 = parking_spot_pair_[1].GetDirection();
    return (atan2(dir1.y, dir1.x) + atan2(dir2.y, dir2.x)) / 2;
  }
  return pose->heading();
}

void ParkingSpotDetection::CalculateParkingVertices(
    double angle, ParkingSpot& parking_spot)
{
  // 方向向量
  Point2D dir(cos(angle), sin(angle));
  // top->down向量
  Point2D top_down_dir(-sin(angle), cos(angle));
  // 边缘线段在方向向量上投影距离
  LineSegment line(parking_spot_pair_[0].GetEnd(), parking_spot_pair_[1].GetStart());
  double proj_dist = line.ProjectLength(angle);
  // 角点
  Point2D spot_left_top, spot_left_down, spot_right_top, spot_right_down;
  if(parking_spot_type_ == ParkingSpotType::PARALLEL_PARKING){
    // 1号点以line1的终点为基准+delta
    double delta = (proj_dist - ParkingSpotParams().spot_length_threshold) / 2;
    spot_left_top = parking_spot_pair_[0].GetEnd() + dir * delta;
    // 0号点
    spot_right_top = spot_left_top + dir * ParkingSpotParams().spot_length_threshold;
    // 2号点
    spot_left_down = spot_left_top + top_down_dir * ParkingSpotParams().spot_width_threshold;
    // 3号点
    spot_right_down = spot_right_top + top_down_dir * ParkingSpotParams().spot_width_threshold;
  } else if(parking_spot_type_ == ParkingSpotType::VERTICAL_PLOT){
    // 1号点以line1的终点为基准+delta
    double delta = (proj_dist - ParkingSpotParams().spot_width_threshold) / 2;
    spot_left_top = parking_spot_pair_[0].GetEnd() + dir * delta;
    // 0号点
    spot_right_top = spot_left_top + dir * ParkingSpotParams().spot_width_threshold;
    // 2号点
    spot_left_down = spot_left_top + top_down_dir * ParkingSpotParams().spot_length_threshold;
    // 3号点
    spot_right_down = spot_right_top + top_down_dir * ParkingSpotParams().spot_length_threshold;
  }
  parking_spot.set_spot_left_top_x(spot_left_top.x);
  parking_spot.set_spot_left_top_y(spot_left_top.y);

  parking_spot.set_spot_left_down_x(spot_left_down.x);
  parking_spot.set_spot_left_down_y(spot_left_down.y);

  parking_spot.set_spot_right_down_x(spot_right_down.x);
  parking_spot.set_spot_right_down_y(spot_right_down.y);

  parking_spot.set_spot_right_top_x(spot_right_top.x);
  parking_spot.set_spot_right_top_y(spot_right_top.x);

  parking_spot.set_spot_type(parking_spot_type_);
}

double ParkingSpotDetection::GrowLineSegment(
    const LineSegment &line, int key, bool flag,
    const ParkingSpotParams& parking_spot_params,
    const std::shared_ptr<Pose> &pose)
{
  LineSegment line_bak = line;
  // 投影线段长度
  double project_length = line.ProjectLength(pose->heading());
  if(flag){
    // line1向后生长
    while(1){
      if(key == 0) return project_length;
      // 相邻线段
      auto pre_line = fit_lines_[--key];
      // 判断两条拟合线段距离是否满足阈值
      if((line_bak.GetStart() - pre_line.GetEnd()).Length() > parking_spot_params.grow_line_dist){
        return project_length;
      // 判断两条拟合线段夹角是否满足阈值
      } else if(line_bak.AngleBetweenLines(pre_line) > parking_spot_params.grow_line_dist){
        return project_length;
      } else {
        // 生长
        project_length += pre_line.ProjectLength(pose->heading());
        // 更新
        line_bak = pre_line;
      }
    }
  } else {
    // line2 向前生长
    key++;
    while(1){
      if(key == static_cast<int>(fit_lines_.size())) return project_length;
      // 相邻线段
      auto next_line = fit_lines_[++key];
      // 判断两条拟合线段距离是否满足阈值
      if((next_line.GetStart() - line_bak.GetEnd()).Length() > parking_spot_params.grow_line_dist){
        return project_length;
      // 判断两条拟合线段夹角是否满足阈值
      } else if(next_line.AngleBetweenLines(line_bak) > parking_spot_params.grow_line_angle * M_PI / 180){
        return project_length;
      } else {
        // 生长
        project_length += next_line.ProjectLength(pose->heading());
        // 更新
        line_bak = next_line;
      }
    }
  }
  // return project_length;
}

bool ParkingSpotDetection::IsCurbLine(
    const LineSegment &line,
    const CurbParams& curb_params,
    const std::shared_ptr<Pose>& pose)
{ 
  // 线段与行驶方向夹角
  double dx = (line.GetEnd() - line.GetStart()).x;
  double dy = (line.GetEnd() - line.GetStart()).y;
  double line_angle = fabs(atan2(dy, dx) - pose->heading());
  // 线段长度和夹角都满足阈值
  if(line_angle >= curb_params.angle_threshold * M_PI / 180 &&
     line.Length() >= curb_params.length_threshold){
    return true;
  }
  return false;
}