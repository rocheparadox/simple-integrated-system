obj-m += simplelindriver.o
simplelindriver-objs := src/simplelindriver.o src/commons.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
