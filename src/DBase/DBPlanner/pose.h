#ifndef DB_PLANNER_POSE_H
#define DB_PLANNER_POSE_H

#include <string>
#include <set>
#include <vector>
using std::string;
using std::set;
using std::vector;
namespace db_planner {
class Pose {
 private:
  string pose_name;
  int pose_id;
  vector<double> object_axis, supporting_surface_axis;
  double lower_bound, upper_bound;

 public:
  void setName(string n) { pose_name = n; }
  void setId(int id) { pose_id = id; }
  void setObjectAxis(vector<double> a) { object_axis = a; }
  void setSupportingSurfaceAxis(vector<double> a) { supporting_surface_axis = a; }
  void setBounds(double l, double u) {lower_bound = l; upper_bound = u;}

  string getPoseName(){ return pose_name; }
  int getId(){ return pose_id; }
  vector<double> getObjectAxis(){ return object_axis; }
  vector<double> getSupportingSurfaceAxis(){ return supporting_surface_axis; }
  double getLowerBound(){ return lower_bound; }
  double getUpperBound(){ return upper_bound; }
};
}

#endif //DB_PLANNER_POSE_H
