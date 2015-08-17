#ifndef _CTYPE_CPTR_SP_
#define _CTYPE_CPTR_SP_

struct cptr_sp_t {
  struct cptr_vec3d_t  linpos_sp ;
  struct cptr_vec3d_t  linvel_sp ;
  struct cptr_vec3d_t  linacc_sp ;
  struct cptr_vec3d_t  angpos_sp ;
  struct cptr_vec3d_t  angvel_sp ;
  struct cptr_vec3d_t  angacc_sp ;

  const char*  type;
};

#endif

