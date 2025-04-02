
CC = gcc
CFLAGS = -O6 -Wall -Wextra -Wpedantic
LDFLAGS = -lmosquitto
TARGET = gqgmctomqtt

##

$(TARGET): $(TARGET).c
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).c $(LDFLAGS)
all: $(TARGET)
clean:
	rm -f $(TARGET)
format:
	clang-format -i $(TARGET).c
prettier:
	prettier --write $(TARGET).js
test: $(TARGET)
	./$(TARGET) ./secrets.txt
.PHONY: all clean format test

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
install: install_target
.PHONY: install install_target

