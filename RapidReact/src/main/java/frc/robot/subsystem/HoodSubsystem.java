    package frc.robot.subsystem;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;

import java.util.function.Supplier;

public class HoodSubsystem extends BitBucketsSubsystem {


    private CANSparkMax motor;
    
    private SparkMaxLimitSwitch m_forwardLimit;
    private SparkMaxLimitSwitch m_reverseLimit;

    boolean lerpShoot = true;




    private final Supplier<Double> revsHub;
    private final Supplier<Double> revsTarmac;
    private final Supplier<Double> revsLaunch;



    private final LerpTable<Double, Double> angleTable = new LerpTable<>();
    private final LerpTable<Double, Double> lowMotorTable = new LerpTable<>();
    private final LerpTable<Double, Double> highMotorTable = new LerpTable<>();

    static final String DESIRED_REVOLUTIONS = "desired revs";
    static final String ACTUAL_REVOLUTIONS = "actual revs";
    static final String DESIRED_ANGLE = "desired angle";

    private final VisionSubsystem visionSubsystem;

    final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, -9.8);


    //845/72 is number of motor turns to raise hood by 1 degree
    private final double HOOD_MOTOR_CONSTANT = 169f/9;

    //TODO
    private final LerpTable<Double, Double> shooterBottom = new LerpTable<>();
    private final LerpTable<Double, Double> shooterTop = new LerpTable<>();


    public HoodSubsystem(Config config, VisionSubsystem visionSubsystem) {
        super(config);

        revsHub = () -> {
            if (this.lerpShoot) {
                return 1.4d;

            }

            return SmartDashboard.getNumber(DESIRED_REVOLUTIONS,0);
        };

        revsTarmac = () -> {
            SmartDashboard.putBoolean("lerpShoot HERE",this.lerpShoot);
            if (this.lerpShoot) {
                return 4d;

            }

            return SmartDashboard.getNumber(DESIRED_REVOLUTIONS,0);
        };

        revsLaunch = () -> {
            if (this.lerpShoot) {
                return 9.5d;

            }

            return SmartDashboard.getNumber(DESIRED_REVOLUTIONS,0);
        };

        this.visionSubsystem = visionSubsystem;
    }


    public void init() {



        SmartDashboard.putNumber(DESIRED_REVOLUTIONS, 0);
        motor = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);

        // Smart Motion Coefficients
        double maxVel = 1500; // rpm
        double  maxAcc = 700;

        RelativeEncoder encoder;
        motor.restoreFactoryDefaults();

        // brushless motors can't be inverted

        // encoder.setInverted(settings.sensorPhase);

        /* Set acceleration and vcruise velocity - see documentation */
        SparkMaxPIDController pidController = motor.getPIDController();

        // configure position PID constants
        pidController.setFF(0);
        pidController.setP(0.0005);
        pidController.setD(0 );
        pidController.setI(0);
        pidController.setIZone(0);
        pidController.setOutputRange(-1, 1);


        pidController.setSmartMotionMaxVelocity(maxVel, 0);
        pidController.setSmartMotionMinOutputVelocity(0, 0);
        pidController.setSmartMotionMaxAccel(maxAcc, 0);
        pidController.setSmartMotionAllowedClosedLoopError(0, 0);


      //  motor.setInverted(true);
        encoder = motor.getEncoder();
        /* Zero the sensor */
        encoder.setPosition(0);

        motor.setIdleMode(IdleMode.kBrake);

        ShuffleboardContainer tab = Shuffleboard.getTab("HoodTesting");
        ;
        SmartDashboard.putNumber(ACTUAL_REVOLUTIONS, motor.getEncoder().getPosition());

        //log shit
        motor.getEncoder().setPosition(0);

        
        this.m_forwardLimit = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        this.m_reverseLimit = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_forwardLimit.enableLimitSwitch(true);
        m_reverseLimit.enableLimitSwitch(true);




        //LERP Table Values
        //angleTable.put(distance, REVOLUTION THINGY)

        //hub (default limelight pos)
        angleTable.put(3.104d,0d);
        lowMotorTable.put(3.104,3500d);
        highMotorTable.put(3.104d,2125d);

        //tarmac
        angleTable.put(2.153d,4d);
        lowMotorTable.put(2.153,3500d);
        highMotorTable.put(2.153,2125d);

        //launchpad
        angleTable.put(6.096,9.5d);
        lowMotorTable.put(6.096d,3500d);
        highMotorTable.put(6.096d,3500d);
    }

    @Override
    public void periodic() {

//        m_forwardLimit.enableLimitSwitch(SmartDashboard.getBoolean("Forward Limit Enabled", false));
//        m_reverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Reverse Limit Enabled", false));
//        SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
//        SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
//
//        if(m_forwardLimit.isLimitSwitchEnabled()){
//            m_forwardLimit.enableLimitSwitch(true);
//        }
//        if(m_reverseLimit.isLimitSwitchEnabled()){
//            m_reverseLimit.enableLimitSwitch(true);
//        }


        //double revs = angleTable.get(visionSubsystem.distance());

        SmartDashboard.putNumber(ACTUAL_REVOLUTIONS, motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42).getPosition());
        SmartDashboard.putNumber("Conversion factor",motor.getEncoder().getPositionConversionFactor());
        SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isPressed());

        SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isPressed());



    }

    @Override
    public void disable() {

    }



    public void setRevsHub() {
        //TODO

        //sets number of rotations of the motor to move the hood by a certain angle parameter
        motor.getPIDController().setReference(revsHub.get(), CANSparkMax.ControlType.kSmartMotion);

    }
    public void setRevsTarmac() {
        //TODO

        //sets number of rotations of the motor to move the hood by a certain angle parameter
        motor.getPIDController().setReference(revsTarmac.get(), CANSparkMax.ControlType.kSmartMotion);

    }

    public void setRevsLaunch() {
        //TODO

        //sets number of rotations of the motor to move the hood by a certain angle parameter
        motor.getPIDController().setReference(revsLaunch.get(), CANSparkMax.ControlType.kSmartMotion);

    }


    public void hoodUp(){
        motor.set(0.1);;
    }


    public void hoodDown(){
        motor.set(-0.1);;
    }

    public void hoodStop(){
        motor.set(0);
    }

    public double getTopShootSpeed() {
        return highMotorTable.get(visionSubsystem.distance());
    }

    public double getBottomShootSpeed() {
        return lowMotorTable.get(visionSubsystem.distance());
    }

    public void toggleLerpShoot() {
        this.lerpShoot = !this.lerpShoot;
        SmartDashboard.putBoolean("lerptoggle", lerpShoot);
    }

}
