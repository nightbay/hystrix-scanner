OBJECTDIR=build/
DISTDIR=dist/
MKDIR=mkdir

OBJECTFILES= \
	${OBJECTDIR}/hystrix-scanner.o \
	${OBJECTDIR}/ccan_json_json.o

DEVOBJECTFILES= \
        ${OBJECTDIR}/hystrix-server.o 


LDLIBSOPTIONS=-lgpiod

${DISTDIR}all: ${OBJECTFILES} ${DEVOBJECTFILES}
	${MKDIR} -p ${DISTDIR}
	$(LINK.c) -o ${DISTDIR}hystrix-scanner ${OBJECTFILES} ${LDLIBSOPTIONS} ${LDFLAGS}
	$(LINK.c) -o ${DISTDIR}hystrix-server ${DEVOBJECTFILES} ${LDFLAGS}

${OBJECTDIR}/hystrix-scanner.o: hystrix-scanner.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) ${CFLAGS} -MMD -MP -MF $@.d -o ${OBJECTDIR}/hystrix-scanner.o hystrix-scanner.c

${OBJECTDIR}/hystrix-server.o: hystrix-server.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) ${CFLAGS} -MMD -MP -MF $@.d -o ${OBJECTDIR}/hystrix-server.o hystrix-server.c

${OBJECTDIR}/ccan_json_json.o: ccan_json_json.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) ${CFLAGS} -MMD -MP -MF $@.d -o ${OBJECTDIR}/ccan_json_json.o ccan_json_json.c


.PHONY: clean

clean:
	${RM} -r build
	${RM} -r dist

