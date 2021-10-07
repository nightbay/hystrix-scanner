OBJECTDIR=build/
DISTDIR=dist/
MKDIR=mkdir

OBJECTFILES= \
	${OBJECTDIR}/hystrix-scanner.o \
	${OBJECTDIR}/ccan_json_json.o

LDLIBSOPTIONS=-lgpiod

${DISTDIR}hystrix-scanner: ${OBJECTFILES}
	${MKDIR} -p ${DISTDIR}
	$(LINK.c) -o ${DISTDIR}hystrix-scanner ${OBJECTFILES} ${LDLIBSOPTIONS} ${LDFLAGS}

${OBJECTDIR}/hystrix-scanner.o: hystrix-scanner.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) ${CFLAGS} -MMD -MP -MF $@.d -o ${OBJECTDIR}/hystrix-scanner.o hystrix-scanner.c

${OBJECTDIR}/ccan_json_json.o: ccan_json_json.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) ${CFLAGS} -MMD -MP -MF $@.d -o ${OBJECTDIR}/ccan_json_json.o ccan_json_json.c

.PHONY: clean

clean:
	${RM} -r build
	${RM} -r dist

