from iocbuilder import Device, AutoSubstitution, records, RecordFactory, SetSimulation, Architecture, ModuleBase
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import AsynPort, Asyn
from iocbuilder.modules.calc import Calc
from iocbuilder.modules.motor import MotorLib, basic_asyn_motor, MotorRecord
from iocbuilder.modules.busy import Busy

__all__ = ['GeoBrick']


class Pmac(Device):
    Dependencies = (Asyn, MotorLib)
    AutoInstantiate = True

class DeltaTauCommsPort(AsynPort):
    "AsynPort that communicates with a delta tau motor controller"
    pass


class DeltaTau(AsynPort):
    "Asyn Motor Port that we can attach motor records to"
    pass


class pmacAsynIPPort(DeltaTauCommsPort):
    """This will create an AsynPort connecting to a PMAC or GeoBrick over IP"""
    LibFileList = ['pmacAsynIPPort']
    DbdFileList = ['pmacAsynIPPort']
    _Cards = []

    def __init__(self, name, IP, simulation=None):
        if ':' not in IP:
            IP = IP + ':1025'
        self.IP = IP
        self.name = name
        # init the AsynPort superclass
        self.__super.__init__(name)

    def Initialise(self):
        print '# Create IP Port (PortName, IPAddr)'
        print 'pmacAsynIPConfigure("%(name)s", "%(IP)s")' % \
            self.__dict__

    ArgInfo = makeArgInfo(__init__,
        name   = Simple('Port Name, normally something like BRICK1port', str),
        IP     = Simple('IP address of the geobrick', str),
        simulation   = Simple('IP port to connect to if in simulation mode', str))

# Sim requires python simulator running on simulation port
def pmacAsynIPPort_sim(name, IP, simulation=None, pmacAsynIPPort=pmacAsynIPPort):
    if simulation:
        return pmacAsynIPPort(name, simulation)
SetSimulation(pmacAsynIPPort, pmacAsynIPPort_sim)


class GeoBrick(DeltaTau):
    """This will create an asyn motor port for a GeoBrick that we can attach
    motor records to using the model 3 driver"""
    LibFileList = ['pmacAsynMotorPort']
    DbdFileList = ['pmacAsynMotorPort']
    Dependencies = (Pmac,)
    _Cards = []

    def __init__(self, Port, name = None, NAxes = 8, IdlePoll = 500, MovingPoll = 100):        
        # init a list of groupnames for each pmacCreateCsGroup to add to
        self.CsGroupNamesList = {}
        # First create an asyn IP port to connect to
        self.PortName = Port.DeviceName()
        # Now add self to list of cards
        self.Card = len(self._Cards)
        self._Cards.append(self)
        if name is None:
            name = "BRICK%d" % (self.Card + 1)
        self.name = name
        # Store other attributes
        self.NAxes = NAxes
        self.IdlePoll = IdlePoll
        self.MovingPoll = MovingPoll
        # init the AsynPort superclass
        self.__super.__init__(name)

    # __init__ arguments
    ArgInfo = makeArgInfo(__init__,
        name = Simple('Name to use for the asyn port', str),
        Port       = Ident('pmacAsynIPPort/pmacVmeConfig to connect to', pmacAsynIPPort),
        NAxes      = Simple('Number of axes', int),
        IdlePoll   = Simple('Idle Poll Period in ms', int),
        MovingPoll = Simple('Moving Poll Period in ms', int))

    def Initialise(self):
        print '# Configure Model 3 Controller Driver (Controler Port,Asyn Motor Port, ADDR, Axes, MOVE_POLL, IDLE_POLL)'
        print 'pmacCreateController("%(name)s", "%(PortName)s", 0, %(NAxes)d, %(MovingPoll)d, %(IdlePoll)d)' % self.__dict__
        print '# Configure Model 3 Axes Driver (Controler Port, Axis Count)'
        print 'pmacCreateAxes("%(name)s", %(NAxes)d)' % self.__dict__

class _GeoBrickGlobalControlT(AutoSubstitution):
    """Creates some PVs for global control of the pmac controller, 
    namely global feed rate and axis coordinate system assignment"""
    TemplateFile = "pmacController.template"
    Dependencies = (GeoBrick,)
    
class GeoBrickGlobalControl(_GeoBrickGlobalControlT, Device):
    """Creates some PVs for global control of the pmac controller, 
    namely global feed rate and axis coordinate system assignment
    IMPORTANT - add this after pmacCreateCSGroup"""
    def __init__(self, **args):        
        self.__super.__init__(**args)
        self.__dict__.update(**args)
    
    # Remove from Arginfo, the (AutoSubstitution) CSG macros since these will be taken from 
    # instances of pmacCreateCsGroup
    removeThese = [ 'CSG%d' % i for i in range(8) ]    
    # Also Instruct Arginfo that PORT is a Geobrick or PMac Controller Port
    ArgInfo = makeArgInfo(__init__,
                          PORT = Ident ('Underlying PMAC or GeoBrick object', DeltaTau)) + \
                          _GeoBrickGlobalControlT.ArgInfo.filtered(without = removeThese + ['PORT'])  
    
    def Finalise(self):
        # create the args needed for the gui - these are taken from instances of pmacCreateCsGroup
        for i in GeoBrickGlobalControl.removeThese:
            if i in self.PORT.CsGroupNamesList:
                self.args[i] = self.PORT.CsGroupNamesList[i]
                    
class _pmacTrajectoryAxis(AutoSubstitution):
    TemplateFile = 'pmacTrajectoryAxis.template'

class GeoBrickTrajectoryControlT(AutoSubstitution):
    """Creates some PVs for executing trajectory scans on the pmac controller"""
    TemplateFile = "pmacControllerTrajectory.template"
    Dependencies = (GeoBrick,)

    def __init__(self, **args):
        # init the super class
        self.__super.__init__(**args)
        self.axes = []
        NAXES = int(args["NAXES"])
        assert NAXES in range(1,33), "Number of axes (%d) must be in range 1..32" % NAXES
        # for each axis
        for i in range(1, NAXES + 1):
            args["MOTOR"] = i
            # make a _pmacTrajectoryAxis instance
            self.axes.append(
                _pmacTrajectoryAxis(
                    **filter_dict(args, _pmacTrajectoryAxis.ArgInfo.Names())))
GeoBrickTrajectoryControlT.ArgInfo.descriptions["PORT"] = Ident("Delta tau motor controller", DeltaTau)


class pmacDisableLimitsCheck(Device):
    Dependencies = (Pmac,)

    def __init__(self, Controller, Axis = None):
        self.__super.__init__()
        self.Controller = Controller
        self.Axis = Axis

    def Initialise(self):
        if self.Axis is None:
            self.Axis = 0
        
        # model 3 version of pmacDisableLimitsCheck uses port instead of card 
        self.ControllerPort = self.Controller.DeviceName() 
        print 'pmacDisableLimitsCheck("%(ControllerPort)s", %(Axis)d, 1)' % self.__dict__

    ArgInfo = makeArgInfo(__init__,
        Controller = Ident ('Underlying PMAC or GeoBrick object', DeltaTau),
        Axis       = Simple('Axis number to disable limit check, defaults to all', int))



def add_basic(cls):
    """Convenience function to add basic_asyn_motor attributes to a class that
    includes it via an msi include statement rather than verbatim"""
    cls.Arguments = basic_asyn_motor.Arguments + [x for x in cls.Arguments if x not in basic_asyn_motor.Arguments]
    cls.ArgInfo = basic_asyn_motor.ArgInfo + cls.ArgInfo.filtered(without=basic_asyn_motor.ArgInfo.Names())
    cls.Defaults.update(basic_asyn_motor.Defaults)
    cls.guiTags = basic_asyn_motor.guiTags
    return cls

class eloss_kill_autohome_records(AutoSubstitution):
    WarnMacros = False
    TemplateFile = "eloss_kill_autohome_records.template"

def add_eloss_kill_autohome(cls):
    """Convenience function to add eloss_kill_autohome_records attributes to a class that
    includes it via an msi include statement rather than verbatim"""
    cls.Arguments = eloss_kill_autohome_records.Arguments + [x for x in cls.Arguments if x not in eloss_kill_autohome_records.Arguments]
    cls.ArgInfo = eloss_kill_autohome_records.ArgInfo + cls.ArgInfo.filtered(without=eloss_kill_autohome_records.ArgInfo.Names())
    cls.Defaults.update(eloss_kill_autohome_records.Defaults)
    cls.guiTags = eloss_kill_autohome_records.guiTags
    return cls

class motor_in_cs_records(AutoSubstitution):
    WarnMacros = False
    TemplateFile = "motor_in_cs.template"

def add_motor_in_cs(cls):
    """Convenience function to add motor_in_cs_records attributes to a class that
    includes it via an msi include statement rather than verbatim"""
    cls.Arguments = motor_in_cs_records.Arguments + [x for x in cls.Arguments if x not in motor_in_cs_records.Arguments]
    cls.ArgInfo = motor_in_cs_records.ArgInfo + cls.ArgInfo.filtered(without=motor_in_cs_records.ArgInfo.Names())
    cls.Defaults.update(motor_in_cs_records.Defaults)
    cls.guiTags = motor_in_cs_records.guiTags
    return cls


@add_basic
@add_eloss_kill_autohome
@add_motor_in_cs
class dls_pmac_asyn_motor(AutoSubstitution, MotorRecord):
    WarnMacros = False
    TemplateFile = 'dls_pmac_asyn_motor.template'
    Dependencies = (Busy,)
    
dls_pmac_asyn_motor.ArgInfo.descriptions["PORT"] = Ident("Delta tau motor controller", DeltaTau)
dls_pmac_asyn_motor.ArgInfo.descriptions["SPORT"] = Ident("Delta tau motor controller comms port", DeltaTauCommsPort)


class autohome(AutoSubstitution):
    Dependencies = (Calc,)
    TemplateFile = 'autohome.template'
autohome.ArgInfo.descriptions["PORT"] = Ident("Delta tau motor controller port", DeltaTau)

class _pmacStatusAxis(AutoSubstitution):
#    ProtocolFiles = ['pmac.proto']
    TemplateFile = 'pmacStatusAxis.template'

class pmacStatus(AutoSubstitution):
    Dependencies = (Pmac,)
#    ProtocolFiles = ['pmac.proto']
    TemplateFile = 'pmacStatus.template'

    def __init__(self, **args):
        # init the super class
        self.__super.__init__(**args)
        self.axes = []
        NAXES = int(args["NAXES"])
        assert NAXES in range(1,33), "Number of axes (%d) must be in range 1..32" % NAXES
        # for each axis
        for i in range(1, NAXES + 1):
            args["AXIS"] = i
            # make a _pmacStatusAxis instance
            self.axes.append(
                _pmacStatusAxis(
                    **filter_dict(args, _pmacStatusAxis.ArgInfo.Names())))
pmacStatus.ArgInfo.descriptions["PORT"] = Ident("Delta tau motor controller", DeltaTau)


class CS(DeltaTau):
    Dependencies = (Pmac,)
    _CSs = []
       
    def __init__(self, name, Controller, CS, PLCNum = None, NAxes = 9, 
        Program = 10, IdlePoll = 500, MovingPoll = 100):

        self.PortName = Controller.PortName
        # PLC number for position reporting
        if PLCNum is None:
            self.PLCNum = CS + 15        
        else:
            self.PLCNum = PLCNum
        # reference for linking pmacAsynCoordCreate and drvAsynMotorConfigure
        self.Ref = len(self._CSs)
        self._CSs.append(self)       
        # Store other attributes
        if name is None:
            name = "CS%d" % (self.Ref)
        self.name = name
        self.NAxes = NAxes
        self.IdlePoll = IdlePoll
        self.MovingPoll = MovingPoll  
        self.Program = Program
        self.CS = CS
        self.Controller = Controller
        # init the AsynPort superclass
        self.__super.__init__(name) 
        print self.__dict__
     
    # __init__ arguments
    ArgInfo = makeArgInfo(__init__,
        name = Simple(
            'CS Name (for asyn port that motor records are connected to)',
            str),
        Controller = Ident ('Underlying PMAC or GeoBrick object', DeltaTau),
        CS         = Simple('CS number', int),
        PLCNum     = Simple('PLC Number, defaults to CS + 15', int),
        NAxes      = Simple('Number of axes', int),
        Program    = Simple('Motion Program to run', int),
        IdlePoll   = Simple('Idle Poll Period in ms', int),
        MovingPoll = Simple('Moving Poll Period in ms', int))
    
    def Initialise(self):
        print '# Create CS (ControllerPort, Addr, CSNumber, CSRef, Prog)'
        print 'pmacCreateCS("%(asyn_name)s", "%(Controller)s", %(CS)d, %(Program)s)' % self.__dict__
        print '# Configure Model 3 CS Axes Driver (Controler Port, Axis Count)'
        print 'pmacCreateCSAxes("%(asyn_name)s", %(NAxes)d)' % self.__dict__
#        print 'pmacAsynCoordCreate("%(PortName)s", 0, %(CS)d, %(Ref)d, %(Program)s)' % self.__dict__
#        print '# Configure CS (PortName, DriverName, CSRef, NAxes)'
#        print 'drvAsynMotorConfigure("%s", "pmacAsynCoord", %d, %d)' % (self.DeviceName(), self.Ref, self.NAxes)                 
#        print '# Set Idle and Moving poll periods (CS_Ref, PeriodMilliSeconds)'
#        print 'pmacSetCoordIdlePollPeriod(%(Ref)d, %(IdlePoll)d)' % self.__dict__
#        print 'pmacSetCoordMovingPollPeriod(%(Ref)d, %(MovingPoll)d)' % self.__dict__

class dls_profile_controller(AutoSubstitution):
    WarnMacros = False
    TemplateFile = 'pmacProfileController.template'
    
class dls_profile_move_axis(AutoSubstitution):
    WarnMacros = False
    TemplateFile = 'pmacProfileAxis.template'
    
class pmacCreateCsGroup(Device):
    """Create a group of axis mappings to coordinate systems. Instantating a GeoBrickGlobalControl
    will create a PV for switching between these groups"""
    Dependencies = (Pmac,)

    def __init__(self, Controller, GroupNumber, GroupName, AxisCount):
        self.__super.__init__()
        self.Controller = Controller
        self.GroupNumber = GroupNumber
        self.AxisCount = AxisCount
        self.GroupName = GroupName

    def Initialise(self):        
        assert type(self.Controller) is GeoBrick or type(self.Controller) is Geobrick, \
            "CsGroup functions are only supported by model 3 drivers Geobrick3, PMAC3"
            # model 3 version of pmacDisableLimitsCheck uses port instead of card 
        print 'pmacCreateCsGroup("%(Controller)s", %(GroupNumber)d, "%(GroupName)s", %(AxisCount)d)' % self.__dict__
        
    def Finalise(self):
        # add groupname to the controller's list
        self.Controller.CsGroupNamesList['CSG%d' % self.GroupNumber] = self.GroupName

    ArgInfo = makeArgInfo(__init__,
        Controller = Ident ('Underlying PMAC3 or GeoBrick3 object', DeltaTau),
        GroupNumber = Simple('Unique Group number to describe this group', int),
        GroupName = Simple('Description of the group', str),
        AxisCount = Simple('Number of CS axes in this group', int))

class pmacCsGroupAddAxis(Device):
    Dependencies = (Pmac,)

    def __init__(self, Controller, GroupNumber, AxisNumber, AxisDef, CoordSysNumber):
        self.__super.__init__()
        self.Controller = Controller
        self.GroupNumber = GroupNumber
        self.AxisNumber = AxisNumber
        self.AxisDef = AxisDef
        self.CoordSysNumber = CoordSysNumber

    def Initialise(self):        
        assert type(self.Controller) is GeoBrick or type(self.Controller) is Geobrick, \
            "CsGroup functions are only supported by model 3 drivers Geobrick3, PMAC3"
            # model 3 version of pmacDisableLimitsCheck uses port instead of card 
        print 'pmacCsGroupAddAxis(%(Controller)s, %(GroupNumber)d, %(AxisNumber)d, %(AxisDef)s, %(CoordSysNumber)d)' % self.__dict__

    ArgInfo = makeArgInfo(__init__,
        Controller = Ident ('Underlying PMAC or GeoBrick object', DeltaTau),
        GroupNumber = Simple('Unique Group number to describe this group', int),
        AxisNumber = Simple('Axis number of axis to add to the group', int),
        AxisDef = Simple('CS Axis definition for this axis i.e. one of I A B C U V W X Y Z (or may include linear equations)', str),
        CoordSysNumber = Simple('Axis number of axis to add to the group', int))

