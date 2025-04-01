
CC = gcc
CFLAGS = -O6 -Wall -pedantic
LDFLAGS = -lmosquitto
TARGET = gqgmctomqtt

$(TARGET): $(TARGET).c
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).c $(LDFLAGS)

all: $(TARGET)

clean:
	rm -f $(TARGET)

format:
	clang-format -i $(TARGET).c

install:
	cp $(TARGET).service /etc/systemd/system/
	systemctl daemon-reload
	systemctl enable $(TARGET)
	systemctl start $(TARGET)

.PHONY: all clean format

