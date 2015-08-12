#ifndef _CTYPE_CPTR_CTRL_PARAMS__
#define _CTYPE_CPTR_CTRL_PARAMS__

struct cptr_ctrl_params_t {
  double l1 ;
  double l2 ;
  double k1 ;
  double k2 ;
  double kp ;
  double kd ;

  double max_thrust     ;
  double max_torque_x   ;
  double max_torque_y   ;
  double max_torque_z   ;
  double hysteresis_thr ;

};

#endif

