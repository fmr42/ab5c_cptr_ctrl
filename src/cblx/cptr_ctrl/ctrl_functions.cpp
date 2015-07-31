#include <armadillo>
using namespace arma;


/*
* Implements the saturate function using the polynomial
*/

vec sat ( const vec& v ) {
	vec v_sat(v.n_elem);
	for ( unsigned int i = 0 ; i<v_sat.n_elem ; i++ ) {
	  if      ( v(i)>+1 )
	    v_sat(i) = +1 ;
	  else if ( v(i)<-1 )
	    v_sat(i) = -1 ;
	  else
	    v_sat(i) = v(i);//1.5 * v(i) - 0.75 * v(i)*v(i)*v(i) + 0.25 * v(i)*v(i)*v(i)*v(i)*v(i) ;
	}
	return v_sat ;
}

