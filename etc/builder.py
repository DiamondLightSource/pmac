from iocbuilder import Device, AutoSubstitution, records, RecordFactory, SetSimulation, Architecture, ModuleBase
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import AsynPort, Asyn
from iocbuilder.modules.motor import MotorLib, basic_asyn_motor, MotorRecord
from iocbuilder.modules.busy import Busy

#__all__ = ['GeoBrick']


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


@add_basic
@add_eloss_kill_autohome
class dls_pmac_asyn_motor(AutoSubstitution, MotorRecord):
    WarnMacros = False
    TemplateFile = 'dls_pmac_asyn_motor.template'
    Arguments = {}
    #ProtocolFiles = ['pmac.proto']
    Dependencies = (Busy,)
    
    def __init__(self, **args):        
        self.__super.__init__(**args)
        self.__dict__.update(**args)

dls_pmac_asyn_motor.ArgInfo.descriptions["PORT"] = Ident("Delta tau motor controller", DeltaTau)
dls_pmac_asyn_motor.ArgInfo.descriptions["SPORT"] = Ident("Delta tau motor controller comms port", DeltaTauCommsPort)



