
#
# sudo apt install libmosquitto-dev
#

##

CC=gcc
CFLAGS_COMMON=\
    -Wfloat-conversion -Werror=float-conversion \
    -Wall -Wextra -Werror -Wpedantic \
    -Wstrict-prototypes -Wold-style-definition \
    -Wno-cast-align -Wcast-qual -Wconversion \
    -Wfloat-equal -Wformat=2 -Wformat-security \
    -Winit-self -Wjump-misses-init \
    -Wlogical-op -Wmissing-include-dirs \
    -Wnested-externs -Wpointer-arith \
    -Wredundant-decls -Wshadow \
    -Wstrict-overflow=2 -Wswitch-default \
    -Wunreachable-code -Wunused \
    -Wwrite-strings \
    -Wdouble-promotion \
    -Wnull-dereference \
    -Wduplicated-cond \
    -Wduplicated-branches \
    -Wrestrict \
    -Wstringop-overflow \
    -Wundef \
    -Wvla \
    -Wno-duplicated-branches \
    -Wno-stringop-truncation
CFLAGS_OPT=-O3
CFLAGS=$(CFLAGS_COMMON) $(CFLAGS_OPT)
LDFLAGS=
LIBS=-lmosquitto

##

TARGET=gqgmctomqtt
MAIN=$(TARGET).c
SOURCES=\
    include/http_linux.h include/mqtt_linux.h include/serial_linux.h \
    include/util_linux.h include/config_linux.h
HOSTNAME:=$(shell hostname)
CFG_SRC:=$(if $(wildcard $(TARGET).$(HOSTNAME).cfg),$(TARGET).$(HOSTNAME).cfg,$(TARGET).cfg)

##

all: $(TARGET)

$(TARGET): $(MAIN) $(SOURCES)
	$(CC) $(CFLAGS) -o $(TARGET) $(MAIN) $(LDFLAGS) $(LIBS)

clean:
	rm -f $(TARGET) $(TARGET).armhf
format:
	clang-format-19 -i $(MAIN) $(SOURCES)
test: $(TARGET)
	./$(TARGET) --config $(CFG_SRC)

DEV_PACKAGES=libmosquitto-dev
DEV_PACKAGES_ARMHF=$(addsuffix :armhf,$(DEV_PACKAGES))
install-dev:
	apt install -y $(DEV_PACKAGES)
remove-dev:
	apt purge -y $(DEV_PACKAGES)
install-dev-armhf:
	dpkg --add-architecture armhf
	apt update
	apt install -y gcc-arm-linux-gnueabihf $(DEV_PACKAGES_ARMHF)
remove-dev-armhf:
	apt purge -y gcc-arm-linux-gnueabihf $(DEV_PACKAGES_ARMHF)
	dpkg --remove-architecture armhf
	apt update

CROSS_CC_ARMHF=arm-linux-gnueabihf-gcc
$(TARGET).armhf: $(MAIN) $(SOURCES)
	$(CROSS_CC_ARMHF) $(CFLAGS) -o $(TARGET).armhf $(MAIN) $(LDFLAGS) $(LIBS)
armhf: $(TARGET).armhf

.PHONY: all clean format test install-dev remove-dev install-dev-armhf remove-dev-armhf armhf

##

INSTALL=$(TARGET)
DIR_INSTALL=/usr/local/bin
DIR_DEFAULT=/etc/default
DIR_SYSTEMD=/etc/systemd/system
define install_service_systemd
	-systemctl stop $(2) 2>/dev/null || true
	-systemctl disable $(2) 2>/dev/null || true
	install -m 644 $(1).service $(DIR_SYSTEMD)/$(2).service
	systemctl daemon-reload
	systemctl enable $(2)
	systemctl start $(2) || echo "Warning: Failed to start $(2)"
endef
install_target: $(TARGET)
	install -m 755 $(TARGET) $(DIR_INSTALL)/$(INSTALL)
install_default: $(CFG_SRC)
	@echo "installing config from $(CFG_SRC)"
	install -m 644 $(CFG_SRC) $(DIR_DEFAULT)/$(INSTALL)
install_service: $(TARGET).service
	$(call install_service_systemd,$(TARGET),$(INSTALL))
install_udev:
	cp 90-*.rules /etc/udev/rules.d/90-$(INSTALL).rules
	udevadm control --reload-rules
	udevadm trigger
install: install_target install_default install_udev install_service
restart:
	systemctl restart $(INSTALL)
.PHONY: install install_target install_default install_udev install_service
.PHONY: restart

##
