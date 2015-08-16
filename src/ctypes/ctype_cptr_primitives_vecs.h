#ifndef _CTYPE_CPTR_PRIMITIVES_VECS_
#define _CTYPE_CPTR_PRIMITIVES_VECS_

struct cptr_scalar_t {
  double data       ;
  char   *unit      ;
  char   *ref_frame ;
  char   *type      ;
} ;

struct cptr_vec3d_t {
  union {
    struct {
      double x;
      double y;
      double z;
    } data;
    double mem[3];
  };
  char *unit      ;
  char *ref_frame ;
  char *type      ;
};

struct cptr_vec4d_t {
  union {
    struct {
      double w;
      double x;
      double y;
      double z;
    } data;
    double mem[3];
  };
  char *unit      ;
  char *ref_frame ;
  char *type      ;
};

#endif

