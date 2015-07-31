struct cptr_sp_lin_pos_t {
  double       data_vec [3] ;
  const char*  type;
  const char*  ref_frame;
};

struct cptr_sp_lin_vel_t {
  double       data_vec [3] ;
  const char*  type;
  const char*  ref_frame;
};

struct cptr_sp_lin_acc_t {
  double       data_vec [3] ;
  const char*  type;
  const char*  ref_frame;
};

struct cptr_sp_ang_pos_t {
  double       data_vec [4] ;
  const char*  type;
  const char*  ref_frame;
};

struct cptr_sp_ang_vel_t {
  double       data_vec [4] ;
  const char*  type;
  const char*  ref_frame;
};

struct cptr_sp_ang_acc_t {
  double       data_vec [4] ;
  const char*  type;
  const char*  ref_frame;
};

struct cptr_sp_t {
  struct cptr_sp_lin_pos_t     cptr_sp_lin_pos ;
  struct cptr_sp_lin_vel_t     cptr_sp_lin_vel ;
  struct cptr_sp_lin_acc_t     cptr_sp_lin_acc ;
  struct cptr_sp_ang_pos_t     cptr_sp_ang_pos ;
  struct cptr_sp_ang_vel_t     cptr_sp_ang_vel ;
  struct cptr_sp_ang_acc_t     cptr_sp_ang_acc ;
  const char*  type;
};

