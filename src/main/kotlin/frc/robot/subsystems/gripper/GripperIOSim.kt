package frc.robot.subsystems.gripper

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class GripperIOSim : GripperIO {
    override val inputs: LoggedGripperInputs = LoggedGripperInputs()
    private var sensorDistance =
        LoggedNetworkNumber("Tuning/Gripper/SensorDistance", 20.0)
    private val motor = TALONFX_SIM
    private val voltageOut = VoltageOut(0.0)

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun updateInputs() {
        inputs.sensorDistance = Units.Centimeters.of(sensorDistance.get())
        inputs.current = motor.appliedCurrent
        inputs.appliedVoltage = motor.appliedVoltage
    }
}
