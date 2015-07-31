struct cptr_state_lin_pos_t {
  double  data_vec [3] ;
  double  covariance [9] ;
  const char*  type;
  const char*  ref_frame;
};

struct cptr_state_lin_vel_t {
  double  data_vec [3] ;
  double  covariance [9];
  const char*  type;
  const char*  ref_frame;
};

struct cptr_state_ang_pos_t {
  double  data_vec [4] ;
  double  covariance [16];
  const char*  type;
  const char*  ref_frame;
};

struct cptr_state_ang_vel_t {
  double  data_vec [4] ;
  double  covariance [16];
  const char*  type;
  const char*  ref_frame;
};

struct cptr_state_t {
  struct cptr_state_lin_pos_t     cptr_state_lin_pos       ;
  struct cptr_state_lin_vel_t     cptr_state_lin_vel       ;
  struct cptr_state_ang_pos_t     cptr_state_ang_pos       ;
  struct cptr_state_ang_vel_t     cptr_state_ang_vel       ;
  bool   hysteresis ;
  const char*  type;
  
};

