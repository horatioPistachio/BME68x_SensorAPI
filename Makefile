all:
	gcc -shared -Wl,-soname,hydro_bme68x -o ../hydroponic_system/bme68x.so -fPIC hydro_bme68x.c bme68x.c bme68x_common.c

test:
	gcc hydro_bme68x.c bme68x.c bme68x_common.c -o test.out

