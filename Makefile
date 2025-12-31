# ===== Toolchain =====
CC      = aarch64-linux-gnu-gcc
CFLAGS  = -Wall -O2 -pthread -Iinclude
LDFLAGS = -pthread -lm

# ===== Output =====
TARGET = app_main

# ===== Source Files =====
SRC = \
    src/daemon2.c \
    src/accident_announce_module.c \
    src/accident_send_module.c \
    src/bluetooth_module.c \
    src/can_interface_module.c \
    src/collision_response_module.c \
    src/collision_risk_module.c \
    src/driving_info_module.c

# ===== Object Files =====
OBJ = $(SRC:.c=.o)

# ===== Build Rules =====
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
