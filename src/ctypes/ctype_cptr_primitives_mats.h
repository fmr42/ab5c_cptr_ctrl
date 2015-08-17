#ifndef _CTYPE_CPTR_PRIMITIVES_MATS_
#define _CTYPE_CPTR_PRIMITIVES_MATS_

// How to specify (and save memory!!)
// that this matrix is symmetric?

struct cptr_vec3d_vec3d_crosscov_t {
  union {
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
    } data;
    double mem[9];
  };
  char *type;
};

struct cptr_vec4d_vec4d_crosscov_t {
  union {
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
    } data;
    double mem[16];
  };
  char *type;
};


struct cptr_vec3d_vec4d_crosscov_t {
  union {
    struct {
      double xw;
      double yw;
      double zw;

      double xx;
      double yx;
      double zx;

      double xy;
      double yy;
      double zy;

      double xz;
      double yz;
      double zz;
    } data;
    double mem[12];
  };
  char *type;
};

/* ======================================== *
 *  Observation models                      *
 * ======================================== */

struct cptr_obsmodel_vec3d_to_vec3d_t{
  union {
    struct {
      double x_to_x;
      double y_to_x;
      double z_to_x;

      double x_to_y;
      double y_to_y;
      double z_to_y;

      double x_to_z;
      double y_to_z;
      double z_to_z;
    } data;
    double mem[9];
  };
  char *type;
};


struct cptr_obsmodel_vec4d_to_vec4d_t{
  union {
    struct {
      double w_to_w;
      double x_to_w;
      double y_to_w;
      double z_to_w;

      double w_to_x;
      double x_to_x;
      double y_to_x;
      double z_to_x;

      double w_to_y;
      double x_to_y;
      double y_to_y;
      double z_to_y;

      double w_to_z;
      double x_to_z;
      double y_to_z;
      double z_to_z;
    } data;
    double mem[16];
  };
  char *type;
};



struct cptr_obsmodel_vec3d_to_vec4d_t{
  union {
    struct {
      double x_to_w;
      double x_to_x;
      double x_to_y;
      double x_to_z;

      double y_to_w;
      double y_to_x;
      double y_to_y;
      double y_to_z;

      double z_to_w;
      double z_to_x;
      double z_to_y;
      double z_to_z;
    } data;
    double mem[12];
  };
  char *type;
};


struct cptr_obsmodel_vec4d_to_vec3d_t{
  union {
    struct {
      double w_to_x;
      double w_to_y;
      double w_to_z;

      double x_to_x;
      double x_to_y;
      double x_to_z;

      double y_to_x;
      double y_to_y;
      double y_to_z;

      double z_to_x;
      double z_to_y;
      double z_to_z;
    } data;
    double mem[12];
  };
  char *type;
};

#endif

