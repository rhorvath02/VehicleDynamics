# FIXME: before you push into master...
RUNTIMEDIR=/usr/bin/../include/omc/c/
#COPY_RUNTIMEFILES=$(FMI_ME_OBJS:%= && (OMCFILE=% && cp $(RUNTIMEDIR)/$$OMCFILE.c $$OMCFILE.c))

fmu:
	rm -f 349.fmutmp/sources/TestRigidChassisBase_init.xml
	cp -a "/usr/bin/../share/omc/runtime/c/fmi/buildproject/"* 349.fmutmp/sources
	cp -a TestRigidChassisBase_FMU.libs 349.fmutmp/sources/

