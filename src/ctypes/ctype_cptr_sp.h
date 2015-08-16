#ifndef _CTYPE_CPTR_SP_
#define _CTYPE_CPTR_SP_


#include "ctype_cptr_primitives_sp.h"

struct cptr_sp_t {
  struct cptr_sp_linpos_t     cptr_sp_linpos ;
  struct cptr_sp_linvel_t     cptr_sp_linvel ;
  struct cptr_sp_linacc_t     cptr_sp_linacc ;
  struct cptr_sp_angpos_t     cptr_sp_angpos ;
  struct cptr_sp_angvel_t     cptr_sp_angvel ;
  struct cptr_sp_angacc_t     cptr_sp_angacc ;
  const char*  type;
};

#endif

