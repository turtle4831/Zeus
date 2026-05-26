import enum
from tokenize import String

from RobotSide.Subsystems.feeder import Feeder, FeederState
from RobotSide.Subsystems.intake import Intake, IntakeState
from RobotSide.Subsystems.multiplexer import Multiplexer
from RobotSide.Subsystems.shooter import Shooter, ShooterState
from RobotSide.Subsystems.swerve import Swerve
from RobotSide.Subsystems.turret import Turret
from RobotSide.Utils.motor import Motor, pidTypes
from RobotSide.Utils.absoluteEncoder import absoluteEncoder
from RobotSide.Utils.beamBreak import BeamBreak
from RobotSide.Utils.hardwareConfig import HardwareConfig
from RobotSide.Utils.pid import PIDController
from RobotSide.Subsystems.swerveModule import SwerveModule


class StateRequest(enum.Enum):
    ACCEPTED = 1
    REJECTED = 2
    PENDING = 3

class StateMachine:
    def __init__(self):
        self.subsystem_states: dict[str, str] = {
            "shooter": "IDLE",
            "feeder": "IDLE",
            "intake": "IDLE",
            "turret": "IDLE",
            "swerve": "IDLE",
        }
    multiplexer1 = Multiplexer(HardwareConfig.MULTIPLEXER1_ID)
    multiplexer2 = Multiplexer(HardwareConfig.MULTIPLEXER2_ID)
    turretEncoder = absoluteEncoder(HardwareConfig.TURRET_ENCODER_ID, multiplexer1)
    turret = Turret(HardwareConfig.TURRET_MOTOR_ID, turretEncoder, 180)
    feeder = Feeder(HardwareConfig.FEEDER_MOTOR_ID, absoluteEncoder(HardwareConfig.FEEDER_ENCODER_ID, multiplexer1), BeamBreak(HardwareConfig.FRONT_BEAM_BREAK_ID), BeamBreak(HardwareConfig.BACK_BEAM_BREAK_ID))
    intake = Intake(HardwareConfig.INTAKE_MOTOR_ID, absoluteEncoder(HardwareConfig.INTAKE_ENCODER_ID, multiplexer1), HardwareConfig.INTAKE_PIVOT_SERVO_ID)
    shooter = Shooter(HardwareConfig.SHOOTER_MOTOR_ID, absoluteEncoder(HardwareConfig.SHOOTER_ENCODER_ID, multiplexer1), HardwareConfig.HOOD_MOTOR_ID, absoluteEncoder(HardwareConfig.HOOD_ENCODER_ID, multiplexer1), speedTolerance=0.05)
    
    frontLeftDriveEncoder = absoluteEncoder(HardwareConfig.FRONT_LEFT_DRIVE_ENCODER_ID, multiplexer1)
    frontLeftTurnEncoder = absoluteEncoder(HardwareConfig.FRONT_LEFT_TURN_ENCODER_ID, multiplexer1)
    frontRightDriveEncoder = absoluteEncoder(HardwareConfig.FRONT_RIGHT_DRIVE_ENCODER_ID, multiplexer1)
    frontRightTurnEncoder = absoluteEncoder(HardwareConfig.FRONT_RIGHT_TURN_ENCODER_ID, multiplexer1)
    backLeftDriveEncoder = absoluteEncoder(HardwareConfig.BACK_LEFT_DRIVE_ENCODER_ID, multiplexer1)
    backLeftTurnEncoder = absoluteEncoder(HardwareConfig.BACK_LEFT_TURN_ENCODER_ID, multiplexer1)
    backRightDriveEncoder = absoluteEncoder(HardwareConfig.BACK_RIGHT_DRIVE_ENCODER_ID, multiplexer1)
    backRightTurnEncoder = absoluteEncoder(HardwareConfig.BACK_RIGHT_TURN_ENCODER_ID, multiplexer1)

    #swerve pid contorollers
    frontLeftDrivePIDController = PIDController(1,0,0)
    frontLeftTurnPIDController = PIDController(1,0,0)
    frontRightDrivePIDController = PIDController(1,0,0)
    frontRightTurnPIDController = PIDController(1,0,0)
    backLeftDrivePIDController = PIDController(1,0,0)
    backLeftTurnPIDController = PIDController(1,0,0)
    backRightDrivePIDController = PIDController(1,0,0)
    backRightTurnPIDController = PIDController(1,0,0)
    
    frontLeftDriveMotor = Motor(HardwareConfig.FRONT_LEFT_DRIVE_MOTOR_ID, frontLeftDriveEncoder, frontLeftDrivePIDController, pidTypes.VELOCITY)
    frontLeftTurnMotor = Motor(HardwareConfig.FRONT_LEFT_TURN_MOTOR_ID, frontLeftTurnEncoder, frontLeftTurnPIDController, pidTypes.POSITION)
    frontRightDriveMotor = Motor(HardwareConfig.FRONT_RIGHT_DRIVE_MOTOR_ID, frontRightDriveEncoder, frontRightDrivePIDController, pidTypes.VELOCITY)
    frontRightTurnMotor = Motor(HardwareConfig.FRONT_RIGHT_TURN_MOTOR_ID, frontRightTurnEncoder, frontRightTurnPIDController, pidTypes.POSITION)
    backLeftDriveMotor = Motor(HardwareConfig.BACK_LEFT_DRIVE_MOTOR_ID, backLeftDriveEncoder, backLeftDrivePIDController, pidTypes.VELOCITY)
    backLeftTurnMotor = Motor(HardwareConfig.BACK_LEFT_TURN_MOTOR_ID, backLeftTurnEncoder, backLeftTurnPIDController, pidTypes.POSITION)
    backRightDriveMotor = Motor(HardwareConfig.BACK_RIGHT_DRIVE_MOTOR_ID, backRightDriveEncoder, backRightDrivePIDController, pidTypes.VELOCITY)
    backRightTurnMotor = Motor(HardwareConfig.BACK_RIGHT_TURN_MOTOR_ID, backRightTurnEncoder, backRightTurnPIDController, pidTypes.POSITION)
    
    frontLeftModule = SwerveModule(frontLeftDriveMotor, frontLeftTurnMotor);
    frontRightModule = SwerveModule(frontRightDriveMotor, frontRightTurnMotor);
    backLeftModule = SwerveModule(backLeftDriveMotor, backLeftTurnMotor);
    backRightModule = SwerveModule(backRightDriveMotor, backRightTurnMotor);
   
    swerve = Swerve(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
        
    blueTargetLocation = list((9.0,9.0))
    redTargetLocaiton = list((-9.0,-9.0))

    def get_all_motors(self) -> list[Motor]:
        return [
            self.turret.motor,
            self.feeder.motor,
            self.intake.motor,
            self.shooter.shooterMotor,
            self.shooter.hoodMotor,
            self.frontLeftDriveMotor,
            self.frontLeftTurnMotor,
            self.frontRightDriveMotor,
            self.frontRightTurnMotor,
            self.backLeftDriveMotor,
            self.backLeftTurnMotor,
            self.backRightDriveMotor,
            self.backRightTurnMotor,
        ]

    def initialize_all_escs(self) -> None:
        for motor in self.get_all_motors():
            motor.initializeESC()

    def update(self, requestedStates:dict[str, str],redAlliance:bool, swerveRequest:list):
        stateRequests: dict[str, StateRequest] = {}

        currentPosition = list(self.swerve.getPose())
        gyroAngle = self.swerve.getGyroAngle()
        if swerveRequest:
            self.swerve.update(gyroAngle)
            self.swerve.fieldCentricDrive(swerveRequest[0], swerveRequest[1], swerveRequest[2], gyroAngle)

        if redAlliance:
            targetPosition = self.redTargetLocaiton
        else:
            targetPosition = self.blueTargetLocation

        self.turret.aimAtPoint(currentPosition, targetPosition)


        self.turret.update()

        for subsystem, state in requestedStates.items(): #if state is requested stop, it needs to stop immediately
            stateRequests[subsystem] = StateRequest.PENDING
            if state != self.subsystem_states[subsystem] and state == "STOP":
                self.subsystem_states[subsystem] = "STOP"
                stateRequests[subsystem] = StateRequest.ACCEPTED

        if self.feeder.getBallCount() >= 1 and requestedStates["shooter"] == ("SHOOTING" or "PRE_SHOOTING"):
            self.feeder.setState(FeederState.IDLE)
            self.shooter.setState(ShooterState.PRE_SHOOTING)
            stateRequests["feeder"] = StateRequest.REJECTED
        
        if self.shooter.atSpeed() and requestedStates["shooter"] == "SHOOTING":
            self.shooter.setState(ShooterState.SHOOTING)
            stateRequests["shooter"] = StateRequest.ACCEPTED

        elif requestedStates["shooter"] == "PRE_SHOOTING":
            self.shooter.setState(ShooterState.PRE_SHOOTING)
            stateRequests["shooter"] = StateRequest.ACCEPTED

        if requestedStates["intake"] == "INTAKE" and stateRequests["intake"] != StateRequest.REJECTED:
            self.intake.setState(IntakeState.INTAKE)
            stateRequests["intake"] = StateRequest.ACCEPTED

        if requestedStates["intake"] == "OUTTAKE" and stateRequests["intake"] != StateRequest.REJECTED:
            self.intake.setState(IntakeState.OUTTAKE)
            stateRequests["intake"] = StateRequest.ACCEPTED

        if requestedStates["intake"] == "IDLE" and stateRequests["intake"] != StateRequest.REJECTED:
            self.intake.setState(IntakeState.IDLE)
            stateRequests["intake"] = StateRequest.ACCEPTED

        if requestedStates["intake"] == "PIVOT_UP" and stateRequests["intake"] != StateRequest.REJECTED:
            self.intake.setState(IntakeState.PIVOT_UP)
            stateRequests["intake"] = StateRequest.ACCEPTED

        

        
            
            
        
        
        
        

    