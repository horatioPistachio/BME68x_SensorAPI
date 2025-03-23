all:
	gcc -shared -Wl,-soname,hydro_bme68x -o bme688.so -fPIC hydro_bme68x.c bme68x.c bme68x_common.c

test:
	gcc hydro_bme68x.c bme68x.c bme68x_common.c -o test.out

