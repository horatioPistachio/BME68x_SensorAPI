all:
	gcc -shared -Wl,-soname,bme68x -o bme68x.so -fPIC bme68x.c

test:
	gcc hydro_bme68x.c bme68x.c bme68x_common.c -o test.out

