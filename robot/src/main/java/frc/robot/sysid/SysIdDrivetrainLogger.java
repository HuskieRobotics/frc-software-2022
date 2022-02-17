package frc.robot.sysid;

public class SysIdDrivetrainLogger extends SysIdLogger {
    double motorVoltage;
    
    public double getMotorVoltage() {
        return motorVoltage;
      }
      
    public void log(double frontLeftPosition, double frontRightPosition, double backLeftPosition, double backRightPosition,
          double frontLeftRate, double frontRightRate, double backLeftRate, double backRightRate,
          double measuredAngle, double angularRate) {
        updateData();
        m_data.add(m_timestamp);
        m_data.add(motorVoltage); // I think this needs to repeat for each of the motors....
        m_data.add(motorVoltage);
        m_data.add(motorVoltage);
        m_data.add(motorVoltage);
        m_data.add(frontLeftPosition);
        m_data.add(frontRightPosition);
        m_data.add(backLeftPosition);
        m_data.add(backRightPosition);
        m_data.add(frontLeftRate);
        m_data.add(frontRightRate);
        m_data.add(backLeftRate);
        m_data.add(backRightRate);
        m_data.add(measuredAngle);
        m_data.add(angularRate);
        motorVoltage = (m_rotate ? -1 : 1) * m_motorVoltage;
      }
      
      void reset() {
        super.reset();
        motorVoltage = 0;
      }
      
      Boolean isWrongMechanism() {
        return !m_mechanism.equals("Drivetrain") && !m_mechanism.equals("Drivetrain (Angular)");
      }
    
}
