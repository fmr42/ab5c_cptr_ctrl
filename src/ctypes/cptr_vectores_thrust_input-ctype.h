// ctype used to store the input to the system.
// Data represent the thrust and the torque generated by the propellers
// in the copter ref frame.
//
struct cptr_vectored_thrust_input_t {
  double thrust     ;
  double torque [3] ;
  char* type;
};

