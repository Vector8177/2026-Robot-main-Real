package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandXboxControllerSim extends CommandXboxController{
    enum XboxButtonPorts{
        LEFT_JOYSTICK_X(0),
        LEFT_JOYSTICK_Y(1),
        RIGHT_JOYSTICK_X(2),
        RIGHT_JOYSTICK_Y(3),
        RIGHT_TRIGGER(4),
        LEFT_TRIGGER(5),
        A(0),
        B(1),
        X(3),
        Y(4),
        LEFT_BUMPER(6),
        RIGHT_BUMPER(7);


        private final int port;
        private XboxButtonPorts(int port){
            this.port = port;
        }

        public int getPort(){
            return port;
        }
    }
    public CommandXboxControllerSim(int port0){
        super(port0);
    }

    @Override
    public double getLeftX() {
        return getRawAxis(XboxButtonPorts.LEFT_JOYSTICK_X.getPort());
    }

    @Override
    public double getLeftY() {
        return getRawAxis(XboxButtonPorts.LEFT_JOYSTICK_Y.getPort());
    }

    @Override
    public double getRightX() {
        return getRawAxis(XboxButtonPorts.RIGHT_JOYSTICK_X.getPort());
    }

    @Override
    public double getRightY() {
        return getRawAxis(XboxButtonPorts.RIGHT_JOYSTICK_Y.getPort());
    }


    @Override
    public Trigger rightTrigger() {
        // TODO Auto-generated method stub
        return axisGreaterThan(XboxButtonPorts.RIGHT_TRIGGER.getPort(), 0);
    }

    @Override
    public Trigger leftTrigger() {
        // TODO Auto-generated method stub
        return axisGreaterThan(XboxButtonPorts.LEFT_TRIGGER.getPort(), 0);
    }

    @Override
    public Trigger a() {
        return button(XboxButtonPorts.A.getPort());
    }

    @Override
    public Trigger b() {
        return button(XboxButtonPorts.B.getPort());
    }

        @Override
    public Trigger x() {
        return button(XboxButtonPorts.X.getPort());
    }

        @Override
    public Trigger y() {
        return button(XboxButtonPorts.Y.getPort());
    }

    @Override
    public Trigger leftBumper() {
        return button(XboxButtonPorts.LEFT_BUMPER.getPort());
    }

    @Override
    public Trigger rightBumper() {
        return button(XboxButtonPorts.RIGHT_BUMPER.getPort());
    }
}
