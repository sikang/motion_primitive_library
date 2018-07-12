#include <mpl_traj_solver/traj_solver.h>
#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

// Pass the data into a VoxelMapUtil class for collision checking
// Plot the result in svg image
typedef boost::geometry::model::d2::point_xy<double> point_2d;
std::ofstream svg("output.svg");
// Declare a stream and an SVG mapper
boost::geometry::svg_mapper<point_2d> mapper(svg, 1000, 1000);

void drawTraj(const Trajectory2D& traj, std::string traj_name, std::string traj_color) {
  // Draw the trajectory
  double total_t = traj.getTotalTime();
  printf("%s: \n", traj_name.c_str());
  printf("   T: %f\n", total_t);
  printf("   J(VEL) = %f, J(ACC) = %f, J(JRK) = %f, J(SNP) = %f\n",
         traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::JRK), traj.J(Control::SNP));
  int num = 200; // number of points on trajectory to draw
  const auto ws = traj.sample(num);
  boost::geometry::model::linestring<point_2d> line;
  for (const auto& w: ws)
    line.push_back(point_2d(w.pos(0), w.pos(1)));
  mapper.add(line);
  mapper.map(line, traj_color);
}

int main(int argc, char **argv) {
  // Draw the canvas
  boost::geometry::model::polygon<point_2d> bound;
  const double origin_x = -1;
  const double origin_y = -1;
  const double range_x = 7;
  const double range_y = 3;
  std::vector<point_2d> points;
  points.push_back(point_2d(origin_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y));
  boost::geometry::assign_points(bound, points);
  boost::geometry::correct(bound);

  mapper.add(bound);
  mapper.map(bound, "fill-opacity:1.0;fill:rgb(255,255,255);stroke:rgb(0,0,0);"
                    "stroke-width:2"); // White

  // Draw path and trajectories
	vec_Vec2f path;
	path.push_back(Vec2f(0, 0));
  path.push_back(Vec2f(1, 0));
  path.push_back(Vec2f(2, 1));
  path.push_back(Vec2f(5, 1));

  // Min Vel Traj
  {
    std::string color = "opacity:0.4;fill:none;stroke:rgb(237,10,63);stroke-width:5"; // Red
    TrajSolver2D traj_solver(Control::VEL);
    traj_solver.setPath(path);
    traj_solver.setV(1); // set velocity for time allocation
    drawTraj(traj_solver.solve(), "min_vel_traj", color);
  }
  // Min Acc Traj
  {
    std::string color = "opacity:0.4;fill:none;stroke:rgb(94,140,49);stroke-width:5"; // Green
    TrajSolver2D traj_solver(Control::ACC);
    traj_solver.setPath(path);
    traj_solver.setV(1); // set velocity for time allocation
    drawTraj(traj_solver.solve(), "min_acc_traj", color);
  }
  // Min Jrk Traj
  {
    std::string color = "opacity:0.4;fill:none;stroke:rgb(118,215,234);stroke-width:5"; // Blue
    TrajSolver2D traj_solver(Control::JRK);
    traj_solver.setPath(path);
    traj_solver.setV(1); // set velocity for time allocation
    drawTraj(traj_solver.solve(), "min_jrk_traj", color);
  }

  // Draw keyframes
  for(const auto& it: path) {
    point_2d pt;
    boost::geometry::assign_values(pt, it(0), it(1));
    mapper.add(pt);
    mapper.map(pt, "fill-opacity:1.0;fill:rgb(255,0,0);", 10); // Red
  }

  // Write title at the lower right corner on canvas
  mapper.text(point_2d(4.0, -0.2), "test_traj_solver",
              "fill-opacity:1.0;fill:rgb(10,10,250);");

  mapper.text(point_2d(3.5, -0.4), "Red: ",
              "fill-opacity:1.0;fill:rgb(237,10,63);");
  mapper.text(point_2d(4.0, -0.4), "minimum velocity trajectory",
              "fill-opacity:1.0;fill:rgb(0,0,0);");

  mapper.text(point_2d(3.5, -0.6), "Green: ",
              "fill-opacity:1.0;fill:rgb(94,140,49);");
  mapper.text(point_2d(4.0, -0.6), "minimum acceleration trajectory",
              "fill-opacity:1.0;fill:rgb(0,0,0);");

  mapper.text(point_2d(3.5, -0.8), "Blue: ",
              "fill-opacity:1.0;fill:rgb(118,215,234);");
  mapper.text(point_2d(4.0, -0.8), "minimum jerk trajectory",
              "fill-opacity:1.0;fill:rgb(0,0,0);");



  return 0;
}
