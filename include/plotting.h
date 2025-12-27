#include "plotrenderer.h"

class Course;
class PlotResult;
struct RiderConfig;

PlotResult run_plot_simulation(const Course& course,
                               const std::vector<RiderConfig>& riders,
                               int target_uid);
