#include "parking_spot_detection.h"

void ParkingSpotDetection::ParkingSpotSearch(
    const Point2D& point, const Pose &pose,
    std::vector<Point2D>& parking_vertices){
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
    // 满足车位再判断的线段对
    if(!ReEvaluateParallelParkingSpot(pair, pose)) continue;

    // 3.车位约束条件判断
    if(!CheckParkingSpotConstraints()) continue;

    // 4.车位位姿计算
    double angle = CalculateParkingSpotAngle(parking_spot_, pose);
    // 5.泊车位角点计算
    CalculateParkingVertices(parking_spot_, angle, parking_vertices);
  }
}

void ParkingSpotDetection::FindPotentialParkingSpots(
    const vector<LineSegment> &fit_lines, const Pose &pose,
    std::map<int, std::vector<LineSegment>>& line_segments){
  // 筛选出路牙线段和障碍物线段
  for (size_t i = 0; i < fit_lines.size() - 1; ++i) {
    const LineSegment& line = fit_lines[i];
    // 判断是否为路牙线段
    if(IsCurbLine(line, pose)){
      curb_lines_.emplace_back(line);
    } else {
      fit_lines_.emplace_back(line);
    }
  }
  for(size_t i = 0; i < fit_lines.size() - 1; ++i){
    LineSegment& line1 = fit_lines[i];
    const LineSegment& line2 = fit_lines[i + 1];
    // 判断特征线段长度是否满足阈值
    if (line1.Length() < LineFitParams().min_fit_distance ||
        line2.Length() < LineFitParams().min_fit_distance) continue;
    // 线段近端点组成新的线段
    LineSegment line(line1.GetEnd(), line2.GetStart());
    // 计算投影长度
    double proj_dist = line.ProjectLength();

    // 判断近端点距离是否大于车位宽度阈值
    std::vector<LineSegment> lines;
    if (proj_dist > ParkingSpotParams().spot_width_threshold) {
      lines.emplace_back(line1);
      lines.emplace_back(line2);
      line_segments[i] = lines;
      // 无路牙时不成立
      // if(curb_lines_map_.find(i + 1) != curb_lines_map_.end()){
      //   parking_space_type_ = ParkingSpaceType::PARALLEL_PARKING;
      // } else {
      //   parking_space_type_ = ParkingSpaceType::VERTICAL_PLOT;
      // }
    }
  }
}

// 对存在潜在车位的线段对再判断
bool ParkingSpotDetection::ReEvaluateParallelParkingSpot(
    const std::map<int, std::vector<LineSegment>> &line_segment_pair,
    const Pose &pose){
  // 通过key找前后线段
  int key = line_segment_pair.first;
  const LineSegment line1 = line_segment_pair.second.front();
  const LineSegment line2 = line_segment_pair.second.back();

  // 生长后线段的投影累计长度
  double proj_length1 = GrowLineSegment(line1, key, true, pose);
  double proj_length2 = GrowLineSegment(line2, key, false, pose);

  // 判断投影长度是否大于长度阈值
  if(proj_length1 > ParkingSpotParams().line_length_threshold &&
     proj_length2 > ParkingSpotParams().line_length_threshold){
    parking_space_type_ = ParkingSpaceType::PARALLEL_PARKING;
    parking_spot_ = {line1, line2};
    return true;
  // 判断投影长度是否大于宽度阈值
  } else if(proj_length1 > ParkingSpotParams().line_width_threshold &&
            proj_length2 > ParkingSpotParams().line_width_threshold){
    parking_space_type_ = ParkingSpaceType::VERTICAL_PLOT;
    parking_spot_ = {line1, line2};
    return true;
  }

  AINFO << "ReEvaluate Parallel Parking Spot Failed.";
  return false;
}

bool ParkingSpotDetection::CheckParkingSpotConstraints(){
  if(parking_spot_.empty()){
    return false;
  }
  const LineSegment& line1 = parking_spot_.first;
  const LineSegment& line2 = parking_spot_.second;

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
    length = line.ProjectLength();
    // 横向偏移
    offset = std::max(offset, LineSegment().DistanceToLine(line1.GetStart(), line2));
    offset = std::max(offset, LineSegment().DistanceToLine(line1.GetEnd(), line2));
    offset = std::max(offset, LineSegment().DistanceToLine(line2.GetStart(), line1));
    offset = std::max(offset, LineSegment().DistanceToLine(line2.GetStart(), line1));
    // 偏移角度
    angle = AngleBetweenLines(line1, line2);
  }
  if(offset > ParkingSpotParams().offset_threshold && angle > angle_threshold){
    return false;
  }
  if(parking_space_type_ == ParkingSpaceType::PARALLEL_PARKING){
    if(depth < spot_width_threshold && length < spot_length_threshold){
      return true;
    }
  } else if(parking_space_type_ == ParkingSpaceType::VERTICAL_PLOT){
    if(depth < spot_length_threshold && length < spot_width_threshold){
      return true;
    }
  }
  return false;
}

double ParkingSpotDetection::CalculateParkingSpotAngle(const Pose& pose){
  if(!curb_lines_.empty()){
    // 有路牙线段用路牙线段计算
    Point2D dir = curb_lines_[0].GetDirection();
    return atan2(dir.y, dir.x);
  } else {
    // 用特征线段
    Point2D dir1 = parking_spot_.first.GetDirection();
    Point2D dir2 = parking_spot_.second.GetDirection();
    return (atan2(dir1.y, dir1.x) + atan2(dir2.y, dir2.x)) / 2;
  }
  return pose.heading();
}

void ParkingSpotDetection::CalculateParkingVertices(
    double angle, std::vector<Point2D> &parking_vertices)
{
  // 方向向量
  Point2D dir(cos(angle), sin(angle));
  // top->down向量
  Point2D top_down_dir(-sin(angle), cos(angle));
  // 边缘线段在方向向量上投影距离
  LineSegment line(parking_spot_.first.GetEnd(), parking_spot_.second.GetStart());
  double proj_dist = line.ProjectLength(angle);
  // 角点
  Point2D spot_left_top, spot_left_down, spot_right_top, spot_right_down;
  if(parking_space_type_ == ParkingSpaceType::PARALLEL_PARKING){
    // 1号点以line1的终点为基准+delta
    double delta = (proj_dist - ParkingSpotParams::spot_length_threshold) / 2;
    spot_left_top = parking_spot_.first.GetEnd() + delta * dir;
    // 0号点
    spot_right_top = spot_left_top + ParkingSpotParams::spot_length_threshold * dir;
    // 2号点
    spot_left_down = spot_left_top + ParkingSpotParams::spot_width_threshold * top_down_dir;
    // 3号点
    spot_right_down = spot_right_top + ParkingSpotParams::spot_width_threshold * top_down_dir;
  } else if(parking_space_type_ == ParkingSpaceType::VERTICAL_PLOT){
    // 1号点以line1的终点为基准+delta
    double delta = (proj_dist - ParkingSpotParams::spot_width_threshold) / 2;
    spot_left_top = parking_spot_.first.GetEnd() + delta * dir;
    // 0号点
    spot_right_top = spot_left_top + ParkingSpotParams::spot_width_threshold * dir;
    // 2号点
    spot_left_down = spot_left_top + ParkingSpotParams::spot_length_threshold * top_down_dir;
    // 3号点
    spot_right_down = spot_right_top + ParkingSpotParams::spot_length_threshold * top_down_dir;
  }
  parking_vertices.emplace_back(spot_right_top);
  parking_vertices.emplace_back(spot_left_top);
  parking_vertices.emplace_back(spot_left_down);
  parking_vertices.emplace_back(spot_right_down);
}

double ParkingSpotDetection::GrowLineSegment(
    const LineSegment &line, int key,
    bool flag, const Pose &pose)
{
  LineSegment line_bak = line;
  // 投影线段长度
  double project_length = line.ProjectLength(pose.heading());
  if(flag){
    // line1向后生长
    while(1){
      if(key == 0) return project_length;
      // 相邻线段
      auto pre_line = fit_lines_[--key];
      // 判断两条拟合线段距离是否满足阈值
      if((line_bak.GetStart() - pre_line.GetEnd()).Length() > ParkingSpotParams().line_dist){
        return project_length;
      // 判断两条拟合线段夹角是否满足阈值
      } else if(AngleBetweenLines(line_bak, pre_line) > ParkingSpotParams().line_angle){
        return project_length;
      } else {
        // 生长
        project_length += pre_line.ProjectLength(pose.heading());
        // 更新
        line_bak = pre_line;
        pre_lines.emplace_back()
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
      if((next_line.GetStart() - line_bak.GetEnd()).Length() > ParkingSpotParams().line_dist){
        return project_length;
      // 判断两条拟合线段夹角是否满足阈值
      } else if(AngleBetweenLines(next_line, line_bak) > ParkingSpotParams().line_angle){
        return project_length;
      } else {
        // 生长
        project_length += next_line.ProjectLength(pose.heading());
        // 更新
        line_bak = next_line;
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

bool ParkingSpotDetection::IsCurbLine(
    const LineSegment &line,
    const Pose& pose)
{ 
  // 线段与行驶方向夹角
  double dx = (line.GetEnd() - line.GetStart()).x;
  double dy = (line.GetEnd() - line.GetStart()).y;
  line_angle = fabs(atan2(dy, dx) - pose.heading());
  // 线段长度和夹角都满足阈值
  if(line_angle >= CurbParams().angle &&
     line.Length() >= CurbParams().length){
    return true;
  }
  return false;
}
