from networktables import NetworkTables
import wpilib
from wpilib.drive import DifferentialDrive
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from wpilib.shuffleboard import Shuffleboard
class MyRobot(wpilib.IterativeRobot):
    def robotInit(self):
        """Robot initialization function"""

        # object that handles basic drive operations
        self.left = wpilib.Victor(0)
        self.right = wpilib.Victor(1)
        self.gyro = wpilib.AnalogGyro(0)
        self.gyro.reset()
        self.myRobot = DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)
        NetworkTables.initialize(server='127.0.0.1')
        self.smartdash = NetworkTables.getTable('SmartDashboard')
        self.gearshift = wpilib.DoubleSolenoid(0,0,1)
        wpilib.CameraServer.launch("vision.py:main")
        self.ll = NetworkTables.getTable("limelight")
        self.ll.putNumber('ledStatus',1)
        # joysticks 1 & 2 on the driver station
        self.leftStick = wpilib.Joystick(2)
        self.rightStick = wpilib.Joystick(1)
        self.trigger = ButtonDebouncer(self.rightStick,1,period=.5)
        self.smartdash.putNumber('tx',1)
        self.gearshift.set(1)
        self.pdp = wpilib.PowerDistributionPanel(0)
        self.accelerometer = wpilib.BuiltInAccelerometer()
        self.gyro.initSendable
        self.myRobot.initSendable
        self.gearshift.initSendable
        self.pdp.initSendable
        self.accelerometer.initSendable
        self.acc = wpilib.AnalogAccelerometer(3)
        self.setpoint = 90.0
        self.P = .3
        self.I = 0
        self.D = 0

        self.integral = 0
        self.previous_error = 0
        self.rcw = 0
        #wpilib.DriverStation.reportWarning(str(self.myRobot.isRightSideInverted()),False)


    def PID(self):
        error = self.setpoint - self.gyro.getAngle()
        self.integral = self.integral + (error*.02)
        derivative = (error - self.previous_error) / .02
        self.rcw = self.P*error + self.I*self.integral + self.D*derivative

    def autonomousInit(self):
        self.myRobot.setSafetyEnabled(False)

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        self.myRobot.arcadeDrive(self.rightStick.getY(), -self.rightStick.getZ() * .4)
        self.smartdash.putNumber('gyrosensor', self.gyro.getAngle())
        if(self.rightStick.getRawButton(4)):
            self.ll.putNumber('ledMode',1)
        if(self.rightStick.getRawButton(3)):
           self.right.set(.5)
        if(self.trigger.get()):
            if(self.gearshift.get() == 1):
                self.gearshift.set(2)
            elif(self.gearshift.get() == 2):
                self.gearshift.set(1)
        if(self.rightStick.getRawButton(2)):
            self.tx = self.ll.getNumber("tx",None)
            if(not type(self.tx) == type(0.0)):
                self.tx = 0.0
            if(self.tx > 1.0):
                self.myRobot.arcadeDrive(0,self.tx * .03)
            elif(self.tx < -1.0):
                self.myRobot.tankDrive(0,self.tx * .03)
            else: 
                self.myRobot.tankDrive(0,0)
        self.pdp.getVoltage()





if __name__ == "__main__":
    wpilib.run(MyRobot)
