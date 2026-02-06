package frc.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.CommandXboxControllerSim.XboxButtonPorts;

public class CommandPS5XboxControllerSim extends CommandXboxController {
        enum PS5ButtonPorts{
        LEFT_JOYSTICK_X(0),
        LEFT_JOYSTICK_Y(1),
        RIGHT_JOYSTICK_X(2),
        RIGHT_JOYSTICK_Y(5),


        RIGHT_TRIGGER(8),
        LEFT_TRIGGER(7),
        A(2),
        B(3),
        X(1),
        Y(4),
        LEFT_BUMPER(5),
        RIGHT_BUMPER(6);


        private final int port;
        private PS5ButtonPorts(int port){
            this.port = port;
        }

        public int getPort(){
            return port;
        }
    }

    public CommandPS5XboxControllerSim(int port){
        super(port);
    }

    @Override
    public double getLeftX() {
        return getRawAxis(PS5ButtonPorts.LEFT_JOYSTICK_X.getPort());
    }

    @Override
    public double getLeftY() {
        return getRawAxis(PS5ButtonPorts.LEFT_JOYSTICK_Y.getPort());
    }

    @Override
    public double getRightX() {
        return getRawAxis(PS5ButtonPorts.RIGHT_JOYSTICK_X.getPort());
    }

    @Override
    public double getRightY() {
        return getRawAxis(PS5ButtonPorts.RIGHT_JOYSTICK_Y.getPort());
    }


    @Override
    public Trigger rightTrigger() {
        // TODO Auto-generated method stub
        return button(PS5ButtonPorts.RIGHT_TRIGGER.getPort());
    }

    @Override
    public Trigger leftTrigger() {
        // TODO Auto-generated method stub
        return button(PS5ButtonPorts.LEFT_TRIGGER.getPort());
    }

    @Override
    public Trigger a() {
        return button(PS5ButtonPorts.A.getPort());
    }

    @Override
    public Trigger b() {
        return button(PS5ButtonPorts.B.getPort());
    }

 @Override
public Trigger x() {
    return button(PS5ButtonPorts.X.getPort())
        .onTrue(new InstantCommand(() -> System.out.println("X button pressed!")));
}

        @Override
    public Trigger y() {
        return button(PS5ButtonPorts.Y.getPort());
    }

    @Override
    public Trigger leftBumper() {
        return button(PS5ButtonPorts.LEFT_BUMPER.getPort());
    }

    @Override
    public Trigger rightBumper() {
        return button(PS5ButtonPorts.RIGHT_BUMPER.getPort());
    }
}
