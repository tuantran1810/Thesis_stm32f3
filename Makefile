BINARY = main
INC_DIR = -IRTOS/include/ -Iinc/
CFLAGS=-c -Wall $(INC_DIR)
CSTD = -std=gnu99

LDSCRIPT = ../stm32f3-discovery.ld

OBJS += RTOS/croutine.o RTOS/list.o RTOS/port.o RTOS/queue.o 
OBJS += RTOS/stream_buffer.o RTOS/tasks.o RTOS/timers.o RTOS/portable/MemMang/heap_3.o
OBJS += src/BAP_setup.o src/BAP_task.o src/BAP_UART.o src/pid_controller.o src/BAP_motor.o

include ../../Makefile.include
