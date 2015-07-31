struct copter_model_t {
  double mass          ;
  double inertia [9]   ;

//  double n_propeller ; // only 4 are supported
//  char   frame_type  ; // X or T, only T is supported
  double arms_length   ;
  double sampling_time ;

  double proc_noise_lin_pos ;
  double proc_noise_lin_vel ;
  double proc_noise_ang_pos ; 
  double proc_noise_ang_vel ;

  double gravity ;

  //double propeller_max_thrust ;
  //double n_propeller          ;

  double max_thrust    ;
  double max_torque[3] ;
  
  char* type ; 
};

