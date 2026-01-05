obj-m +=reciever_driver.o

ifdef M
	ccflags-y += -I"$(M)/include"
endif

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD)
modules_install:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install
dt: nrf24-overlay.dts
	dtc -@ -I dts -O dtb -o nrf24.dtbo nrf24-overlay.dts
	sudo cp nrf24.dtbo /boot/overlays/
clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build  M=$(PWD) clean
	rm -rf nrf24-overlay.dtbo
