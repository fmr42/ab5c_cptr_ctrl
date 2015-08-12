#ifndef _CTYPE_CPTR_PRIMITIVES_
#define _CTYPE_CPTR_PRIMITIVES_

union vec3d_t {
  struct {
    double x;
    double y;
    double z;
  } ;
  double mem[3];
};

union vec3d_cov_t {
  // How to specify (and save memory!!)
  // that this matrix is symmetric?
  struct {
    double xx;
    double yx;
    double zx;
    double xy;
    double yy;
    double zy;
    double xz;
    double yz;
    double zz;
  };
  double mem[9];
};

union quat_t {
  struct {
    double w;
    double x;
    double y;
    double z;
  } ;
  double mem[4];
};

union quat_cov_t {
  // How to specify (and save memory!!)
  // that this matrix is symmetric?
  struct {
    double ww;
    double xw;
    double yw;
    double zw;
    double wx;
    double xx;
    double yx;
    double zx;
    double wy;
    double xy;
    double yy;
    double zy;
    double wz;
    double xz;
    double yz;
    double zz;
  };
  double mem[9];
};

struct cptr_state_lin_pos_t {
  union vec3d_t     vec ;
  union vec3d_cov_t  cov ;
  const char *unit      ;
  const char *ref_frame ;
  const char *type      ;
};

struct cptr_state_lin_vel_t {
  union vec3d_t     vec ;
  union vec3d_cov_t  cov ;
  const char *unit      ;
  const char *ref_frame ;
  const char *type      ;
};

struct cptr_state_ang_pos_t {
  union quat_t      vec ;
  union quat_cov_t   cov ;
  const char *ref_frame ;
  const char *type      ;
};

struct cptr_state_ang_vel_t {
  union quat_t      vec ;
  union quat_cov_t   cov ;
  const char *ref_frame ;
  const char *type      ;
};

struct cptr_sp_lin_pos_t {
  union vec3d_t     vec ;
  const char *unit      ;
  const char *ref_frame ;
  const char *type      ;
};

struct cptr_sp_lin_vel_t {
  union vec3d_t     vec ;
  const char *unit      ;
  const char *ref_frame ;
  const char *type      ;
};

struct cptr_sp_lin_acc_t {
  union vec3d_t     vec ;
  const char *unit      ;
  const char *ref_frame ;
  const char *type      ;
};

struct cptr_sp_ang_pos_t {
  union quat_t      vec ;
  union quat_cov_t   cov ;
  const char *ref_frame ;
  const char *type      ;
};

struct cptr_sp_ang_vel_t {
  union quat_t      vec ;
  union quat_cov_t   cov ;
  const char *ref_frame ;
  const char *type      ;
};

struct cptr_sp_ang_acc_t {
  union quat_t      vec ;
  union quat_cov_t   cov ;
  const char *ref_frame ;
  const char *type      ;
};

#endif

