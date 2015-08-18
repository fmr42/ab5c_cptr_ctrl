default:
	g++ test_001_predict.cpp -c -o test_001_predict.o 
	g++ test_001_predict.o ../../cblx/cptr_ekf_predict/cptr_ekf_predict.o ../../lib/arma_quaternion_lib.o -o test_001_predict -larmadillo


justlink:
	g++ test_001_predict.o ../../cblx/cptr_ekf_predict/cptr_ekf_predict.o ../../lib/arma_quaternion_lib.o -o test_001_predict -larmadillo

clean:
	rm test_001_predict.o -f
	rm test_001_predict -f


