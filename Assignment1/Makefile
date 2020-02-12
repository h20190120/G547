obj-m := main.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

exe:
	gcc -o out userapp.c
	./out
