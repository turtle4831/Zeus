import enum

from Zeus.RobotSide.Subsystems.feeder import Feeder, FeederState
from Zeus.RobotSide.Subsystems.intake import Intake, IntakeState
from Zeus.RobotSide.Subsystems.multiplexer import Multiplexer
from Zeus.RobotSide.Subsystems.shooter import Shooter, ShooterState
from Zeus.RobotSide.Subsystems.swerve import Swerve
from Zeus.RobotSide.Subsystems.turret import Turret
from Zeus.RobotSide.Utils.motor import Motor, pidTypes
from Zeus.RobotSide.Utils.absoluteEncoder import absoluteEncoder
from Zeus.RobotSide.Utils.beamBreak import BeamBreak
from Zeus.RobotSide.Utils.hardwareConfig import HardwareConfig
from Zeus.RobotSide.Subsystems.swerveModule import SwerveModule


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
    feeder = Feeder(HardwareConfig.FEEDER_MOTOR_ID, absoluteEncoder(HardwareConfig.MULTIPLEXER1_ID, multiplexer1), BeamBreak(HardwareConfig.FRONT_BEAM_BREAK_ID), BeamBreak(HardwareConfig.MIDDLE_BEAM_BREAK_ID), BeamBreak(HardwareConfig.BACK_BEAM_BREAK_ID))
    intake = Intake(HardwareConfig.INTAKE_MOTOR_ID, absoluteEncoder(HardwareConfig.MULTIPLEXER1_ID, multiplexer1), HardwareConfig.INTAKE_PIVOT_SERVO_ID)
    shooter = Shooter(HardwareConfig.SHOOTER_MOTOR_ID, absoluteEncoder(HardwareConfig.MULTIPLEXER1_ID, multiplexer1), HardwareConfig.HOOD_MOTOR_ID, absoluteEncoder(HardwareConfig.MULTIPLEXER1_ID, multiplexer1), speedTolerance=0.05)
    
    frontLeftDriveEncoder = absoluteEncoder(HardwareConfig.FRONT_LEFT_DRIVE_ENCODER_ID, multiplexer1)
    frontLeftTurnEncoder = absoluteEncoder(HardwareConfig.FRONT_LEFT_TURN_ENCODER_ID, multiplexer1)
    frontRightDriveEncoder = absoluteEncoder(HardwareConfig.FRONT_RIGHT_DRIVE_ENCODER_ID, multiplexer1)
    frontRightTurnEncoder = absoluteEncoder(HardwareConfig.FRONT_RIGHT_TURN_ENCODER_ID, multiplexer1)
    backLeftDriveEncoder = absoluteEncoder(HardwareConfig.BACK_LEFT_DRIVE_ENCODER_ID, multiplexer1)
    backLeftTurnEncoder = absoluteEncoder(HardwareConfig.BACK_LEFT_TURN_ENCODER_ID, multiplexer1)
    backRightDriveEncoder = absoluteEncoder(HardwareConfig.BACK_RIGHT_DRIVE_ENCODER_ID, multiplexer1)
    backRightTurnEncoder = absoluteEncoder(HardwareConfig.BACK_RIGHT_TURN_ENCODER_ID, multiplexer1)

    frontLeftDriveMotor = Motor(HardwareConfig.FRONT_LEFT_DRIVE_MOTOR_ID, frontLeftDriveEncoder, None, pidTypes.VELOCITY)
    frontLeftTurnMotor = Motor(HardwareConfig.FRONT_LEFT_TURN_MOTOR_ID, frontLeftTurnEncoder, None, pidTypes.POSITION)
    frontRightDriveMotor = Motor(HardwareConfig.FRONT_RIGHT_DRIVE_MOTOR_ID, frontRightDriveEncoder, None, pidTypes.VELOCITY)
    frontRightTurnMotor = Motor(HardwareConfig.FRONT_RIGHT_TURN_MOTOR_ID, frontRightTurnEncoder, None, pidTypes.POSITION)
    backLeftDriveMotor = Motor(HardwareConfig.BACK_LEFT_DRIVE_MOTOR_ID, backLeftDriveEncoder, None, pidTypes.VELOCITY)
    backLeftTurnMotor = Motor(HardwareConfig.BACK_LEFT_TURN_MOTOR_ID, backLeftTurnEncoder, None, pidTypes.POSITION)
    backRightDriveMotor = Motor(HardwareConfig.BACK_RIGHT_DRIVE_MOTOR_ID, backRightDriveEncoder, None, pidTypes.VELOCITY)
    backRightTurnMotor = Motor(HardwareConfig.BACK_RIGHT_TURN_MOTOR_ID, backRightTurnEncoder, None, pidTypes.POSITION)
    
    frontLeftModule = SwerveModule(frontLeftDriveMotor, frontLeftTurnMotor);
    frontRightModule = SwerveModule(frontRightDriveMotor, frontRightTurnMotor);
    backLeftModule = SwerveModule(backLeftDriveMotor, backLeftTurnMotor);
    backRightModule = SwerveModule(backRightDriveMotor, backRightTurnMotor);
   
    swerve = Swerve(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
        

    def update(self, requestedStates:dict[str, str]):
        stateRequests: dict[str, StateRequest] = {}

        for subsystem, state in requestedStates.items(): #if state is requested stop, it needs to stop immediately
            stateRequests[subsystem] = StateRequest.PENDING
            if state != self.subsystem_states[subsystem] and state == "STOP":
                self.subsystem_states[subsystem] = "STOP"
                stateRequests[subsystem] = StateRequest.ACCEPTED

        if self.feeder.getBallCount() == 3:
            self.feeder.setState(FeederState.IDLE)
            self.intake.setState(IntakeState.PIVOT_UP)
            self.shooter.setState(ShooterState.PRE_SHOOTING)
            stateRequests["feeder"] = StateRequest.REJECTED
            stateRequests["intake"] = StateRequest.REJECTED
        
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

        

        
            
            
        
        
        
        

    