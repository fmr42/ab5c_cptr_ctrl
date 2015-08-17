#ifndef _CTYPE_CPTR_SENSDATA_
#define _CTYPE_CPTR_SENSDATA_

#include "ctype_cptr_primitives_mats.h"
#include "ctype_cptr_primitives_vecs.h"

struct cptr_sensdata_3d_t {
  struct cptr_vec3d_t                vec;
  struct cptr_vec3d_vec3d_crosscov_t cov;
  char *type;
};

struct cptr_sensdata_4d_t {
  struct cptr_vec4d_t                vec;
  struct cptr_vec4d_vec4d_crosscov_t cov;
  char *type;
};

#endif

