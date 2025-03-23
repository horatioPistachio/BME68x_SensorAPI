import ctypes
testlib = ctypes.CDLL('./hydro_bme68x.so')

testlib.get_bme_reading.restype = ctypes.c_char_p
static_result = testlib.get_bme_reading()
print(f"Static string: {static_result.decode('utf-8')}")
# a= testlib.myprint(2)
# print(a)