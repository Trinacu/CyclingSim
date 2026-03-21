#ifndef GROUPING_PARAMS_H
#define GROUPING_PARAMS_H

typedef struct GroupingParams {
  double gap_threshold = 10.0; // metres — gap to split/form
  double form_gap = 8.0;       // hysteresis: close threshold
  double break_gap = 12.0;     // hysteresis: open threshold
} GroupingParams;

#endif
