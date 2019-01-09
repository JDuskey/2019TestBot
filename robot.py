from networktables import NetworkTables
import wpilib
from wpilib.drive import DifferentialDrive

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
        wpilib.CameraServer.launch("vision.py:main")

        # joysticks 1 & 2 on the driver station
        self.leftStick = wpilib.Joystick(2)
        self.rightStick = wpilib.Joystick(1)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        self.myRobot.tankDrive(self.leftStick.getY(), self.rightStick.getY())
        self.smartdash.putNumber('gyrosensor', self.gyro.getAngle())


if __name__ == "__main__":
    wpilib.run(MyRobot)
