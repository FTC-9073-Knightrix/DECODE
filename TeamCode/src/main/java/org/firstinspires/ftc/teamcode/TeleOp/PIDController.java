package org.firstinspires.ftc.teamcode.TeleOp;

public class PIDController {  /* dont ask me about this math i just asked gpt to write i
                                and did it in my */
    private double kP, kI, kD;
    private double integral, previousError;
    private double setpoint;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.previousError = 0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double measurement) {
        double error = setpoint - measurement;
        integral += error;
        double derivative = error - previousError;
        previousError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }
}

