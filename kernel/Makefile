KVERSION := $(shell uname -r)

obj-m = pmc464.o
pmc464-objs:=pmc464_pci.o

all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
install:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules_install
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean