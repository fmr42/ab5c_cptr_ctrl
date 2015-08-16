#ifndef _CTYPE_CPTR_PRIMITIVES_SP_
#define _CTYPE_CPTR_PRIMITIVES_SP_

#include "ctype_cptr_primitives_vecs.h"

struct cptr_sp_linpos_t {
  struct cptr_vec3d_t     vec ;
  char *type      ;
};

struct cptr_sp_linvel_t {
  struct cptr_vec3d_t     vec ;
  char *type      ;
};

struct cptr_sp_linacc_t {
  struct cptr_vec3d_t     vec ;
  char   *type      ;
};

struct cptr_sp_angpos_t {
  struct cptr_vec4d_t      vec ;
  char *type      ;
};

struct cptr_sp_angvel_t {
  struct cptr_vec4d_t      vec ;
  char *type      ;
};

struct cptr_sp_angacc_t {
  struct cptr_vec4d_t      vec ;
  char *type      ;
};

#endif

