package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax DeployMotor;
    private AbsoluteEncoder DeployEncoder;
    private PIDController DeployPID;
    private double DeploySet;

    private CANSparkMax IntakeMotor;
    private boolean FeedIntake;
    private double IntakeSpeed;
    private double IntakeAngle;

    private SparkLimitSwitch limitSwitch1, limitSwitch2;


    public IntakeSubsystem(){
        DeployMotor = new CANSparkMax(30, MotorType.kBrushless);
        IntakeMotor = new CANSparkMax(31, MotorType.kBrushless);

        DeployEncoder = DeployMotor.getAbsoluteEncoder(Type.kDutyCycle);
        this.DeploySet = 0;

        limitSwitch1 = IntakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

        this.IntakeAngle = 0;
        this.IntakeSpeed = 0;

        DeployPID = new PIDController(0, 0, 0);
    }

    public double getDeployRotPosition(){
        return DeployEncoder.getPosition();
    }

    public void setDeploy(double DeploySet){
        this.DeploySet = DeploySet;
    }

    public double getDeploySet(){
        return this.DeploySet;
    }

    public boolean getIRSensor(){
        return false;
    }

    public void RotateIntake(double angle){
        this.IntakeAngle = angle;
    }

    public void setIntakeSpeed(double IntakeSpeed){
        this.IntakeSpeed = IntakeSpeed;
    }
    
    public double getIntakeSpeed(){
        return this.IntakeSpeed;
    }

    public void FeedIntake(double IntakeSpeed){
       this.FeedIntake = true;
       this.IntakeSpeed = IntakeSpeed;
    }

    public boolean getLimitSwitch(){
        return limitSwitch1.isPressed();
    }

    @Override
    public void periodic(){
        IntakeMotor.set(getIntakeSpeed());

        if(!getIRSensor() && FeedIntake){
            FeedIntake = false;
            setIntakeSpeed(0);
        } 
        
        DeployMotor.set(DeployPID.calculate(getDeployRotPosition(), IntakeAngle));
    }
}

