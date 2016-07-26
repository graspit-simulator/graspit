#ifndef DB_PLANNER_FEATURES_EXTRACTOR_H
#define DB_PLANNER_FEATURES_EXTRACTOR_H

#include "model.h"

namespace db_planner {

template <class Features>
class FeaturesExtractor {
 public:
  virtual bool Extract(const Model& model, Features* features) const {
    return false;
  }
};


}  // end namespace db_planner


#endif  // DB_PLANNER_FEATURES_EXTRACTOR_H