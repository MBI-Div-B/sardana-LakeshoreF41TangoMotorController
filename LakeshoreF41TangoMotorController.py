from sardana import State
from sardana.pool.controller import MotorController
from sardana.pool.controller import Type, Description, DefaultValue

from tango import DeviceProxy
import time

class LakeshoreF41TangoMotorController(MotorController):
    ctrl_properties = {'tangoFQDN': {Type: str,
                                     Description: 'The FQDN of the LKSf41Gaussmeter Tango DS',
                                     DefaultValue: 'domain/family/member'},
                       }
    
    MaxDevice = 1
    
    def __init__(self, inst, props, *args, **kwargs):
        super(MotorController, self).__init__(
            inst, props, *args, **kwargs)

        print('Lakeshore F41 Initialization ...')
        self.proxy = DeviceProxy(self.tangoFQDN)
        print('SUCCESS')
        self._timeout = 10
        self._threshold = 0.002
        self._motors = {}
        
    def AddDevice(self, axis):
        self._motors[axis] = {}
        self._motors[axis]['is_moving'] = None
        self._motors[axis]['move_start_time'] = None
        self._motors[axis]['target'] = None

    def DeleteDevice(self, axis):
        del self._motors[axis]

    def StateOne(self, axis):
        limit_switches = MotorController.NoLimitSwitch
        
        pos = self.ReadOne(axis)
        now = time.time()
        
        try:
            if self._motors[axis]['is_moving'] == False:
                state = State.On
            elif self._motors[axis]['is_moving'] & (abs(pos-self._motors[axis]['target']) > self._threshold): 
                # moving and not in threshold window
                if (now-self._motors[axis]['move_start_time']) < self._timeout:
                    # before timeout
                    state = State.Moving
                else:
                    # after timeout
                    self._log.warning('Lakeshore Timeout')
                    self._motors[axis]['is_moving'] = False
                    state = State.On
            elif self._motors[axis]['is_moving'] & (abs(pos-self._motors[axis]['target']) <= self._threshold): 
                # moving and within threshold window
                self._motors[axis]['is_moving'] = False
                state = State.On
            else:
                state = State.Fault
        except:
            state = State.Fault
        
        return state, 'all fine', limit_switches

    def ReadOne(self, axis):
        return self.proxy.MagneticField

    def StartOne(self, axis, position):
        if self.proxy.OpenLoop != 0:
            print('LKSf41: switching to closed loop')
            self.proxy.OpenLoop = 0
        if self.proxy.FieldControl != 1:
            print('LKSf41: enabling field control')
            self.proxy.FieldControl = 1
        self.proxy.SetpointField = position
        self._motors[axis]['move_start_time'] = time.time()
        self._motors[axis]['is_moving'] = True
        self._motors[axis]['target'] = position

    def StopOne(self, axis):
        return self.proxy.MagneticField

    def AbortOne(self, axis):
        pass
    
    def SendToCtrl(self, cmd):
        """
        Send custom native commands. The cmd is a space separated string
        containing the command information. Parsing this string one gets
        the command name and the following are the arguments for the given
        command i.e.command_name, [arg1, arg2...]
        :param cmd: string
        :return: string (MANDATORY to avoid OMNI ORB exception)
        """
        # Get the process to send
        mode = cmd.split(' ')[0].lower()

        if mode == 'enable':
            self.proxy.FieldControl = 1
        elif mode == 'disable':
            self.proxy.FieldControl = 0
        else:
            self._log.warning('Invalid command')
            return 'ERROR: Invalid command requested.'
        pass
