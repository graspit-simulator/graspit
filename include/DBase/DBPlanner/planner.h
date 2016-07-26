#ifndef DB_PLANNER_PLANNER_H
#define DB_PLANNER_PLANNER_H

#include <string>
#include <vector>
#include "aligner.h"
#include "grasp.h"
#include "grasp_ranker.h"
#include "model.h"
#include "neighbor_finder.h"
#include "training_planner.h"
using std::string;
using std::vector;

namespace db_planner {

template <class Input>
class Planner {
 private:
  const NeighborFinder<Input>& neighbor_finder_;
  const TrainingPlanner& training_planner_;
  const GraspRanker& grasp_selector_;     
  const Aligner<Input>& aligner_;
  const int num_neighbors_;
  vector<Grasp> grasps_;
  vector<Grasp>::iterator current_grasp_;
  const Input& input_;
 public:
  Planner(const NeighborFinder<Input>& neighbor_finder,
          const TrainingPlanner& training_planner,
          const GraspRanker& grasp_ranker,
          const Aligner<Input>& aligner, 
          const int num_neighbors) 
            : neighbor_finder_(neighbor_finder),
              training_planner_(training_planner),
              grasp_ranker_(grasp_ranker),
              aligner_(aligner),
              num_neighbors_(num_neighbors) { }
  // CreatePlan populates the grasps_ vector with un-aligned grasps.
  bool CreatePlan(const Input& input) {
    // We need to store the input_ so we can use it later in alignments.
    input_ = input;
    // Find the k-nearest neighbors.
    vector<string> neighbors(num_neighbors_);
    if (!neighbor_finder_.Find(input_, num_neighbors_, &neighbors)) 
      return false;
    // Get the grasps for those model names
    grasps_.clear();
    for (vector<string>::const_iterator neighbor_name = neighbors.begin();
         neighbor_name != neighbors.end(); ++neighbor_name) {
      vector<Grasp> trained_grasps;
      training_planner_.TrainedGrasps(*neighbor_name, &trained_grasps);
      grasps_.insert(grasps_.end(), 
                     trained_grasps.begin(), 
                     trained_grasps.end());
    }
    // Rank the grasps from those neighbors.
    if (!grasp_ranker_.Rank(&grasps_)) 
      return false;    
    current_grasp_ = grasps_.begin();
    return true;
  }

  // GetAlignedGrasp returns the next grasp from the grasps_ vector aligned
  // with the input. This continues until there are no more grasps. Grasps
  // which fail to be aligned are silently consumed.
  bool NextGrasp(Grasp* grasp) {
    for ( ; current_grasp_ != grasps_.end(); current_grasp_++) {
      float alignemnt[16];
      if (aligner_.Align(*(current_grasp_->SourceModel()), input_, alignemnt)) {
        *grasp = current_grasp_->Transform(alignemnt);
        return true;
      }
    }
    return false;
  }
};


}  // end namespace db_planner


#endif  // DB_PLANNER_PLANNER_H