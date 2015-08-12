#ifndef _CTYPE_CPTR_CTRL_STATE_
#define _CTYPE_CPTR_CTRL_STATE_

struct cptr_ctrl_state_t {
  double hysteresis;
  double Qc0_old[4];
  double Qc1_old[4];
};

#endif

