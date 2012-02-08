IDLC = `rtm-config --idlc`
IDL_INCDIR = `rtm-config --prefix`/include/rtm/idl
IDLS = JointDataTypes.idl RobotHardwareService.idl \
	SequencePlayerService.idl ForceSensor.idl \
	HIROController.idl HiroNX.idl \
	GraspController.idl ArmMotion.idl CoordTransService.idl \
	Img.idl Vision.idl

all: idlcompile

idlcompile: $(IDLS)
	$(IDLC) -bpython -I$(IDL_INCDIR) $^

clean:
	rm -rf *_idl.py *.pyc _GlobalIDL _GlobalIDL__POA OpenHRP OpenHRP__POA RTC RTC__POA Img Img__POA
