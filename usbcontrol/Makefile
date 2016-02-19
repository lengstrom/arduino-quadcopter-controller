CC := clang
CCFLAGS := -fobjc-arc
LDFLAGS := -framework Foundation -framework IOKit

.PHONY: all
all: usbcontrol

.PHONY: clean
clean:
	rm -rf *.o usbcontrol

usbcontrol: usbcontrol.o
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CCFLAGS) -c $< -o $@
