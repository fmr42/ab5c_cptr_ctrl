default: compile link

compile:
	g++ test_002_ctrl.cpp -c -o test_002_ctrl.o 


link:
	g++ test_002_ctrl.o ../../cblx/cptr_ekf_predict/cptr_ekf_predict.o ../../cblx/cptr_ctrl/cptr_ctrl.o ../../lib/arma_quaternion_lib.o ../../lib/cptr_ctrl_aux_fun.o -o test_002_ctrl -larmadillo

clean:
	rm test_002_ctrl.o -f
	rm test_002_ctrl -f


