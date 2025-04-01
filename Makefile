
CC = gcc
CFLAGS = -O6 -Wall -Wextra -Wpedantic
LDFLAGS = -lmosquitto
TARGET = gqgmctomqtt

$(TARGET): $(TARGET).c
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).c $(LDFLAGS)

all: $(TARGET)

clean:
	rm -f $(TARGET)

format:
	clang-format -i $(TARGET).c

install: $(TARGET)
	cp $(TARGET).service /etc/systemd/system/
	systemctl daemon-reload
	systemctl enable $(TARGET)
	systemctl start $(TARGET)

test: $(TARGET)
	./$(TARGET) 

.PHONY: all clean format install test

