#ifndef _CTYPE_CPTR_SP_
#define _CTYPE_CPTR_SP_

struct cptr_sp_t {
  struct cptr_sp_lin_pos_t     cptr_sp_lin_pos ;
  struct cptr_sp_lin_vel_t     cptr_sp_lin_vel ;
  struct cptr_sp_lin_acc_t     cptr_sp_lin_acc ;
  struct cptr_sp_ang_pos_t     cptr_sp_ang_pos ;
  struct cptr_sp_ang_vel_t     cptr_sp_ang_vel ;
  struct cptr_sp_ang_acc_t     cptr_sp_ang_acc ;
  const char*  type;
};

#endif

