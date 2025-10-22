package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase{
    public DcMotorEx intakeMotor;
    public Servo intakeLeftServo;
    public Servo intakeRightServo;
    private final double motorInput = 1.0;
    public Intake(final HardwareMap hardwareMap){ // init
        intakeMotor=hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeLeftServo=hardwareMap.get(Servo.class,"intakeLeft");
        intakeRightServo=hardwareMap.get(Servo.class,"intakeRight");

        intakeRightServo.setDirection(Servo.Direction.REVERSE);
        intakeLeftServo.setDirection(Servo.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void IntakeIn(){  // actions : take the artifact in
        intakeMotor.setPower(motorInput);
        intakeLeftServo.setPosition(1);
        intakeLeftServo.setPosition(1);
    }
    public void IntakeOut(){  //actions : take the artifact out

        intakeMotor.setPower(-motorInput);
        intakeLeftServo.setPosition(0);
        intakeLeftServo.setPosition(0);
    }
    public void IntakeStop(){
        intakeMotor.setPower(0);
        intakeLeftServo.setPosition(0.5);
        intakeLeftServo.setPosition(0.5);
    }
}
