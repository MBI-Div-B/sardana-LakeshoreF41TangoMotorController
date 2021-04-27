from sardana import State
from sardana.pool.controller import MotorController
from sardana.pool.controller import Type, Description, DefaultValue, Access

from tango import DeviceProxy
import time

class LakeshoreF41TangoMotorController(MotorController):
    '''Sardana motor controller for DESY Lakeshore F41 tango device server.

    This controller is for use with an F41 Gaussmeter with the optional field
    control module. It uses two axes:
        0 -> closed loop operation (Tesla)
        1 -> open loop (Ampere)
    '''
    ctrl_properties = {
        'tangoFQDN': {
            Type: str,
            Description: 'The FQDN of the LKSf41Gaussmeter Tango DS',
            DefaultValue: 'domain/family/member'},
    }
    ctrl_attributes = {
        'CL_threshold': {
            Type: float,
            Access: 'read_write',
            Description: 'Accuracy threshold for closed loop moves (Tesla)',
            DefaultValue: 1e-3},
        'OL_waittime': {
            Type: float,
            Access: 'read_write',
            Description: 'Wait time after open loop moves (seconds)',
            DefaultValue: 0.1},
        }
    
    MaxDevice = 2
    
    def __init__(self, inst, props, *args, **kwargs):
        super(MotorController, self).__init__(
            inst, props, *args, **kwargs)

        print('Lakeshore F41 Initialization ...')
        self.proxy = DeviceProxy(self.tangoFQDN)
        self._OL_waittime = 0.1
        self._CL_threshold = 0.001
        print('SUCCESS')
        self._timeout = 10
        self._motors = {}
        
    def AddDevice(self, axis):
        self._motors[axis] = {}
        self._motors[axis]['is_moving'] = False
        self._motors[axis]['move_start_time'] = None
        self._motors[axis]['target'] = None
        self._motors[axis]['threshold_CL'] = 1e-3
        self._motors[axis]['wait_OL'] = 0.1

    def DeleteDevice(self, axis):
        del self._motors[axis]

    def StateOne(self, axis):
        '''Determine motor state.

        In closed loop, timeouts may occur depending on desired accuracy.
        For open loop mode, the actual current value is unknown, so just wait
        for a specified time.
        '''
        limit_switches = MotorController.NoLimitSwitch
        now = time.time()
        target = self._motors[axis]['target']
        start_time = self._motors[axis]['move_start_time']

        try:
            if self._motors[axis]['is_moving'] == False:
                state = State.On
            elif axis == 0:  # closed loop, still moving
                pos = self.ReadOne(axis)
                if abs(pos - target) > self._CL_threshold:  # outside threshold
                    if (now - start_time) < self._timeout:  # no timeout
                        state = State.Moving
                    else:  # timeout
                        self._log.warning('LKSf41 took too long to reach pos.')
                        self.StopOne(axis)
                        state = State.On
                else:  # target reached before timeout
                    self._motors[axis]['is_moving'] = False
                    state = State.On
            else: ## open loop, still moving; no feedback, just wait time
                if (now - start_time) < self._OL_waittime:
                    state = State.Moving
                else:
                    self._motors[axis]['is_moving'] = False
                    state = State.On
        except Exception:
            state = State.Fault
        
        return state, 'all fine', limit_switches

    def ReadOne(self, axis):
        '''Return magnetic field (axis 0) or open loop setpoint (axis 1).
        
        Magnetic field value is valid in both modes, open loop setpoint only
        in open loop mode.
        '''
        if axis == 0:
            return self.proxy.MagneticField
        else:
            if self.proxy.OpenLoop == 1:
                return self.proxy.setpointopenloop
        return None

    def StartOne(self, axis, position):
        self.proxy.FieldControl = 1
        if axis == 0:  # closed loop
            self.proxy.OpenLoop = 0
            self.proxy.SetpointField = position
        else:  # open loop
            self.proxy.OpenLoop = 1
            self.proxy.SetpointOpenLoop = position
        
        self._motors[axis]['move_start_time'] = time.time()
        self._motors[axis]['is_moving'] = True
        self._motors[axis]['target'] = position

    def StopOne(self, axis):
        if axis == 0:
            curr = self.proxy.MagneticField
            self._motors[axis]['target'] = curr
            self._motors[axis]['is_moving'] = False
            return curr
        else:
            return self.proxy.SetpointOpenLoop

    def AbortOne(self, axis):
        pass

    def getCL_threshold(self):
        return self._CL_threshold
    
    def setCL_threshold(self, value):
        self._CL_threshold = value
    
    def getOL_waittime(self):
        return self._OL_waittime

    def setOL_waittime(self, value):
        self._OL_waittime = value
    
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
