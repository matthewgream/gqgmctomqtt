
CC = gcc
CFLAGS = -O6 -Wall -Wextra -Wpedantic
LDFLAGS = -lmosquitto
TARGET = gqgmctomqtt
HOSTNAME = $(shell hostname)

##

$(TARGET): $(TARGET).c
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).c $(LDFLAGS)
all: $(TARGET)
clean:
	rm -f $(TARGET)
format:
	clang-format -i $(TARGET).c include/*.h
test: $(TARGET)
	./$(TARGET) $(TARGET).cfg-$(HOSTNAME)
.PHONY: all clean format test lint

##

SYSTEMD_DIR = /etc/systemd/system
define install_systemd_service
	-systemctl stop $(1) 2>/dev/null || true
	-systemctl disable $(1) 2>/dev/null || true
	cp $(2).service $(SYSTEMD_DIR)/$(1).service
	systemctl daemon-reload
	systemctl enable $(1)
	systemctl start $(1) || echo "Warning: Failed to start $(1)"
endef
install_target: $(TARGET).service
	$(call install_systemd_service,$(TARGET),$(TARGET))
install_udev:
	cp 90-*.rules /etc/udev/rules.d
	udevadm control --reload-rules
	udevadm trigger
install: install_udev install_target
.PHONY: install install_target

