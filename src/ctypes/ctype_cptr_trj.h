#ifndef _CTYPE_CPTR_TRJ_
#define _CTYPE_CPTR_TRJ_


#include "ctype_cptr_primitives_vecs.h"

struct cptr_trj_t {

  // Trajectory code
  unsigned int trj_code          ;

  // Minimum time reired, ie time required
  // if max_linvel, max_linacc, max_angvel and max_angacc
  // are not reached.
  float   min_time               ;

  struct cptr_vec3d_t max_linvel ;
  struct cptr_vec3d_t max_linacc ;

  struct cptr_vec3d_t max_angvel ;
  struct cptr_vec3d_t max_angacc ;

  const char*  type;
};

#endif

