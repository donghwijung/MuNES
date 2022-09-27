#pragma once

struct Pose7D {
  double p_x;
  double p_y;
  double p_z;
  double q_x;
  double q_y;
  double q_z;
  double q_w;
};

struct Loop {
  int prev_node_idx;
  int curr_node_idx;
  Pose7D trans; 
};