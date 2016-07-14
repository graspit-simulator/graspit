#ifndef DB_PLANNER_CACHING_FEATURES_EXTRACTOR_H
#define DB_PLANNER_CACHING_FEATURES_EXTRACTOR_H

#include <string>
#include "db_manager.h"
#include "features_extractor.h"
#include "model.h"
using std::string;
 
namespace db_planner {

// The CachingFeaturesExtractor wraps another FeaturesExtractor and checks into
// a database for precomputed features before attempting extraction. If
// "cache_results_" is true, then new alignments are written back to the
// database. 
template <class Features>
class CachingFeaturesExtractor : public FeaturesExtractor<Features>{
 private:
  const DatabaseFeaturesManager& db_manager_;
  const bool cache_results_;
  const string features_type_name_;
  const FeaturesExtractor<Features>& features_extractor_;
 public:
  CachingFeaturesExtractor(
      const DatabaseFeaturesManager& db_manager,
      const bool cache_results, 
      const string& features_type_name,
      const FeaturesExtractor<Features>& features_extractor 
          = FeaturesExtractor<Features>())
        : db_manager_(db_manager),
          cache_results_(cache_results), 
          features_type_name_(features_type_name),
          features_extractor_(features_extractor) { }

  virtual bool Extract(const Model& model, Features* features) const {
    // Try to retrieve saved features first.
    if (db_manager.GetFeatures(model.ModelName(), 
                               features_type_name_, 
                               features)) {
      return true;
    }
    // That failed, so try to extract with the supplied FeaturesExtractor.
    if (features_extractor_.Extract(model, features)) {
      // If desired, save the results in the database.
      if (cache_results_) {
        db_manager.SaveFeatures(model.ModelName(),
                                features_type_name_,
                                *features);
      }
      return true;
    } 
    return false; // Failure; we can't do this extraction.
  }
};


}  // end namespace db_planner

#endif  // DB_PLANNER_CACHING_FEATURES_EXTRACTOR_H