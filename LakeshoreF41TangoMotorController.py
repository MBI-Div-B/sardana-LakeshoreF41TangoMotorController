from sardana import State
from sardana.pool.controller import MotorController, OneDController
from sardana.pool.controller import Type, Description, DefaultValue, Access
from datetime import datetime

from tango import DeviceProxy
import time


class LakeshoreF41TangoMotorController(MotorController):
    """Sardana motor controller for DESY Lakeshore F41 tango device server.

    This controller is for use with an F41 Gaussmeter with the optional field
    control module. It uses two axes:
        0 -> closed loop operation (Tesla)
        1 -> open loop (Ampere)
    """

    ctrl_properties = {
        "tangoFQDN": {
            Type: str,
            Description: "The FQDN of the LKSf41Gaussmeter Tango DS",
            DefaultValue: "domain/family/member",
        },
    }
    ctrl_attributes = {
        "CL_threshold_rel": {
            Type: float,
            Access: "read_write",
            Description: "Relative accuracy threshold for closed loop moves",
            DefaultValue: 0.02,
        },
        "CL_threshold_abs": {
            Type: float,
            Access: "read_write",
            Description: "Absolute accuracy threshold for closed loop moves (mT)",
            DefaultValue: 0.5,
        },
        "OL_waittime": {
            Type: float,
            Access: "read_write",
            Description: "Wait time after open loop moves (seconds)",
            DefaultValue: 0.2,
        },
    }

    MaxDevice = 2

    def __init__(self, inst, props, *args, **kwargs):
        super(MotorController, self).__init__(inst, props, *args, **kwargs)

        print("Lakeshore F41 Initialization ...")
        self.proxy = DeviceProxy(self.tangoFQDN)
        self._OL_waittime = 0.2
        self._th_rel = 0.02
        self._th_abs = 0.5
        print("SUCCESS")
        self._timeout = 20
        self._motors = {}

    def AddDevice(self, axis):
        self._motors[axis] = {}
        self._motors[axis]["is_moving"] = False
        self._motors[axis]["move_start_time"] = 0
        self._motors[axis]["move_end_time"] = 0
        if axis == 0:
            pos = self.proxy.SetPointField
        else:
            pos = self.proxy.SetPointOpenLoop
        self._motors[axis]["target"] = pos

    def DeleteDevice(self, axis):
        del self._motors[axis]

    def StateOne(self, axis):
        """Determine motor state.

        In closed loop, timeouts may occur depending on desired accuracy.
        For open loop mode, the actual current value is unknown, so just wait
        for a specified time.
        """
        limit_switches = MotorController.NoLimitSwitch
        now = time.time()
        target = self._motors[axis]["target"]
        start_time = self._motors[axis]["move_start_time"]

        try:
            if self._motors[axis]["is_moving"] == False:
                state = State.On
            elif axis == 0:  # closed loop, still moving
                pos = self.ReadOne(axis)
                diff_abs = abs(pos - target)
                try:
                    diff_rel = diff_abs / target
                except ZeroDivisionError:
                    diff_rel = 1
                if (diff_rel > self._th_rel) and (
                    diff_abs > self._th_abs
                ):  # outside threshold
                    if (now - start_time) < self._timeout:  # no timeout
                        state = State.Moving
                    else:  # timeout
                        self._log.warning("LKSf41 took too long to reach pos.")
                        self.StopOne(axis)
                        state = State.On
                else:  # target reached before timeout
                    self._motors[axis]["is_moving"] = False
                    state = State.On
            else:  ## open loop, still moving; no feedback, just wait time
                # if (now - start_time) < self._OL_waittime:
                if now < self._motors[axis]["move_end_time"]:
                    state = State.Moving
                else:
                    self._motors[axis]["is_moving"] = False
                    state = State.On
        except Exception:
            state = State.Fault

        return state, "all fine", limit_switches

    def ReadOne(self, axis):
        """Return magnetic field (axis 0) or open loop setpoint (axis 1).

        Magnetic field value is valid in both modes, open loop setpoint only
        in open loop mode.
        """
        if axis == 0:
            return 1e3 * self.proxy.MagneticField
        else:
            if self.proxy.OpenLoop == 1:
                return self.proxy.setpointopenloop
        return None

    def StartOne(self, axis, position):
        if axis == 0:  # closed loop
            self.proxy.OpenLoop = 0
            self.proxy.SetpointField = 1e-3 * position
            self.proxy.FieldControl = 1
        else:  # open loop
            self.proxy.OpenLoop = 1
            self.proxy.SetpointOpenLoop = position
            self.proxy.FieldControl = 1

        now = time.time()
        self._motors[axis]["move_start_time"] = now
        duration = 3 * abs(self._motors[axis]["target"] - position)
        self._motors[axis]["move_end_time"] = min(max(1, duration), 10) + now
        self._motors[axis]["is_moving"] = True
        self._motors[axis]["target"] = position

    def StopOne(self, axis):
        if axis == 0:
            curr = 1e3 * self.proxy.MagneticField
            self._motors[axis]["target"] = curr
            self._motors[axis]["is_moving"] = False
            return curr
        else:
            return self.proxy.SetpointOpenLoop

    def AbortOne(self, axis):
        pass

    def getCL_threshold_rel(self):
        return self._th_rel

    def setCL_threshold_rel(self, value):
        self._th_rel = value

    def getCL_threshold_abs(self):
        return self._th_abs

    def setCL_threshold_abs(self, value):
        self._th_abs = value

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
        mode = cmd.split(" ")[0].lower()

        if mode == "enable":
            self.proxy.FieldControl = 1
        elif mode == "disable":
            self.proxy.FieldControl = 0
        else:
            self._log.warning("Invalid command")
            return "ERROR: Invalid command requested."
        pass


class LakeshoreF41TangoCounterController(OneDController):
    """
    Collect a trace of magnet field values at fixed interval.

    Provides two axes: field values and corresponding time stamps
    """

    MaxDevice = 2

    ctrl_properties = {
        "socket_device": {
            Type: str,
            Description: "Tango FQDN of the socket DS used for communication",
            DefaultValue: "domain/family/member",
        },
    }

    def __init__(self, inst, props, *args, **kwargs):
        super().__init__(inst, props, *args, **kwargs)
        self.socket = DeviceProxy(self.socket_device)
        self._npts = 1
        self._acquire = False
        self.field_values = []
        self.timestamps = []

    def GetAxisPar(self, axis, par):
        if par == "shape":
            return (self._npts,)

    def StateOne(self, axis):
        if not self._acquire:
            return State.On, "Ready to acquire."
        else:
            self.fetch_data()
            if len(self.field_values) >= self._npts:
                self._acquire = False
                return State.On, "Ready to acquire."
            return State.Moving, "Acquiring data."

    def ReadOne(self, axis):
        if axis == 0:
            return self.field_values[: self._npts]
        if axis == 1:
            return self.timestamps[: self._npts]

    def LoadOne(self, axis, value, repetitions, latency):
        self._npts = int(value / 0.2)  # 200 ms per sample

    def StartOne(self, axis):
        pass

    def StartAll(self):
        self.socket.write("FETC:BUFF:CLE")
        self.timestamps = []
        self.field_values = []
        self._acquire = True

    def StopOne(self, axis):
        self._acquire = False

    def fetch_data(self, bufstr):
        bufstr = self.socket.writeandread("FETC:BUFF:DC?")
        lines = bufstr.strip('";\r\n').split(";")
        values = np.array([line.split(",") for line in lines])
        tstamps = [datetime.fromisoformat(v) for v in values[:, 0]]
        tstamps = [datetime.timestamp(t) for t in tstamps]
        fields = [float(v) for v in values[:, 1]]
        self.timestamps.append(tstamps)
        self.field_values.append(fields)
        return
