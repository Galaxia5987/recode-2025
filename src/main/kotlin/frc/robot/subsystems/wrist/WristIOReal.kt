package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TorqueCurrentConfigs
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage

class WristIOReal : WristIO {
    override val inputs: LoggedWristInputs = LoggedWristInputs()
    private val motionMagic = MotionMagicTorqueCurrentFOC(0.0).withSlot(0)
    private val voltageOut = VoltageOut(0.0)
    private val motor = TalonFX(15)
    private val absoluteEncoder = CANcoder(50)

    private val softLimits =
        SoftwareLimitSwitchConfigs().apply {
            ForwardSoftLimitEnable = true
            ReverseSoftLimitEnable = true
            ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT.rotations
            ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT.rotations
        }

    init {
        motor.configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput =
                    MotorOutputConfigs().apply {
                        NeutralMode = NeutralModeValue.Brake
                        Inverted = InvertedValue.CounterClockwise_Positive
                    }

                Feedback =
                    FeedbackConfigs().apply {
                        RotorToSensorRatio = ROTOR_TO_SENSOR
                        SensorToMechanismRatio = SENSOR_TO_MECHANISM
                        FeedbackRemoteSensorID = 50
                        FeedbackSensorSource =
                            FeedbackSensorSourceValue.FusedCANcoder
                    }

                Slot0 =
                    Slot0Configs().apply {
                        kP = 150.0
                        kI = 0.0
                        kD = 8.0
                        kG = 0.0
                        GravityType = GravityTypeValue.Arm_Cosine
                        StaticFeedforwardSign =
                            StaticFeedforwardSignValue.UseClosedLoopSign
                    }

                MotionMagic =
                    MotionMagicConfigs().apply {
                        MotionMagicCruiseVelocity = 100.0
                        MotionMagicAcceleration = 200.0
                    }

                TorqueCurrent =
                    TorqueCurrentConfigs().apply {
                        PeakForwardTorqueCurrent = 30.0
                        PeakReverseTorqueCurrent = -30.0
                    }

                SoftwareLimitSwitch = softLimits
                CurrentLimits =
                    CurrentLimitsConfigs().apply {
                        StatorCurrentLimitEnable = true
                        StatorCurrentLimit = 20.0
                        SupplyCurrentLimitEnable = true
                        SupplyCurrentLimit = 40.0
                    }
            }
        )

        absoluteEncoder.configurator.apply(
            CANcoderConfiguration().apply {
                MagnetSensor.SensorDirection =
                    SensorDirectionValue.Clockwise_Positive
                MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9
                MagnetSensor.MagnetOffset = 0.0
            }
        )
        absoluteEncoder.setPosition(0.0)
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(motionMagic.withPosition(angle))
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun resetAbsoluteEncoder(angle: Angle) {
        absoluteEncoder.setPosition(angle)
    }

    override fun updateInputs() {
        inputs.angle = motor.position.value
        inputs.appliedVoltage = motor.motorVoltage.value
    }
}
