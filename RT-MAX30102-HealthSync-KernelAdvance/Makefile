# Makefile: Builds the MAX30102 driver module.
# Supplemented with install and clean targets.
# No changes to original code, only added comments.

# Object module.
obj-m += max30102_driver.o  # Add driver object.

# Objects composing the driver.
max30102_driver-objs := max30102_core.o max30102_i2c.o max30102_interrupt.o max30102_config.o max30102_data.o max30102_ioctl.o max30102_advanced_kernel.o max30102_mmap.o  # List all object files.

# All target: Build modules.
all:  # All target.
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules  # Build in kernel dir.

# Clean target: Clean build files.
clean:  # Clean target.
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean  # Clean in kernel dir.
	rm -f *.ko *.mod.c *.o *.mod.o Module.symvers modules.order .*.cmd  # Remove files.
	rm -rf .tmp_versions  # Remove temp dir.

# Install target: Build and insert module.
install: all  # Install depends on all.
	sudo insmod max30102_driver.ko  # Insert module.