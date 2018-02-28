INC_PATH=-Iinclude/ -I../../../../../../libopencm3/include
LIB_PATH=
LIBS=
RUNTIME_PATH=
CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar
CFLAGS=-Wall $(INC_PATH) -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

SOURCES+=$(wildcard *.c)
SOURCES+=$(wildcard portable/MemMang/heap_3.c)

OBJECTS=$(patsubst %.c, %.o, $(SOURCES))

EXECUTABLE=rtos.a

all: $(EXECUTABLE)

$(EXECUTABLE): build $(OBJECTS)
	$(AR) $(CFLAGS) -c -o $@ $(RUNTIME_PATH) $(OBJECTS) $(LIB_PATH) $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< $(LIB_PATH) $(LIBS) -o $@

build:
	@mkdir -p bin

clean:
	rm -rf $(EXECUTABLE) $(OBJECTS)
	find . -name "*~" -exec rm {} \;
	find . -name "*.o" -exec rm {} \;
	find . -name "*.d" -exec rm {} \;