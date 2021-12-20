OS := $(shell uname -s)

ifeq (Linux,${OS})
CC := cc
WARNFLAG := -Wall
LIBFLAGS := 
endif

ifeq (SCO_SV,${OS})
CC := cc
WARNFLAG := -w3 -wX
LIBFLAGS := -lm
endif

ifeq (SunOS,${OS})
CC := gcc
WARNFLAG := -Wall
LIBFLAGS := -lm -lelf
endif

SRCS := $(wildcard *.c)
OBJS := $(SRCS:.c=.o)
PROGS := $(SRCS:.c=)
CFLAGS := -D ${OS} -O ${WARNFLAG}

SCRIPTS := find_osc setparms titan-readuart titan-setbaud

%: %.c
	${CC} ${CFLAGS} -o $@ $< ${LIBFLAGS}

all: ${PROGS}

clean:
	@rm -rf ${OBJS} ${PROGS}

fixperms:
	@for i in ${SCRIPTS}; do	\
		chmod a+x $$i;		\
	done
