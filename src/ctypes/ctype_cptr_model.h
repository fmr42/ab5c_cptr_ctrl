#ifndef _CTYPE_CPTR_MODEL_
#define _CTYPE_CPTR_MODEL_


struct cptr_model_t {
  double mass          ;
  double inertia [9]   ;

//  double n_propeller ; // only 4 are supported
//  char   frame_type  ; // X or T, only T is supported
  double arms_length   ;

  double proc_noise_linpos ;
  double proc_noise_linvel ;
  double proc_noise_angpos ; 
  double proc_noise_angvel ;

  double gravity ;

  //double propeller_max_thrust ;
  //double n_propeller          ;

  double max_thrust    ;
  double max_torque[3] ;
  
  char* type ; 
};

#endif

