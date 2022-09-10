CROSS_COMPILE = arm-linux-gnueabihf-
ARCH=arm
# CC=i586-poky-linux-gcc
# CC=arm-linux-gnueabihf-gcc-8
#	Compiler
CC = gcc

#	compiler flags:
#	-Wall turn on compiler warnings
CFLAGS  = -Wall

#	Option to attach POSIX Thread Library
OPTION	= -lgpiod -lrt

# the build target executable:
TARGET = assignment2

# APP= testcc

KDIR=/lib/modules/$(shell uname -r)/build

obj-m = cycle_count_mod.o

ccflags-m += -Wall

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules
	$(CC) $(CFLAGS) $(TARGET).c -o $(TARGET) $(OPTION)
	
	
clean:
	make ARCH=x86 CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) clean
	$(RM) $(TARGET)
