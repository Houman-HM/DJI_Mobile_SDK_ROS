package com.dji.ros_dji;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.ros.android.MessageCallable;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.Timer;
import java.util.TimerTask;
import java.util.function.DoubleBinaryOperator;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.FlightOrientationMode;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.Attitude;
import dji.common.gimbal.GimbalMode;
import dji.common.gimbal.GimbalState;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.util.CommonCallbacks;
import dji.keysdk.BatteryKey;
import dji.keysdk.DJIKey;
import dji.keysdk.KeyManager;
import dji.keysdk.FlightControllerKey;
import dji.keysdk.GimbalKey;
import dji.keysdk.callback.GetCallback;
import dji.keysdk.callback.KeyListener;
import dji.keysdk.KeyManager;
import dji.keysdk.callback.ActionCallback;
import dji.keysdk.callback.GetCallback;
import dji.keysdk.callback.KeyListener;
import dji.keysdk.callback.SetCallback;
import dji.sdk.base.BaseProduct;
import dji.sdk.flightcontroller.FlightAssistant;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;
import std_msgs.Float32;
import std_msgs.Float64;
import std_msgs.Int32;
import std_msgs.String;

class Ros_Publishers extends AbstractNodeMain {
    public float Gimbal_Roll=0,Gimbal_Pitch=0,Gimbal_Yaw=0,Velocity_X_Drone=0,Velocity_Y_Drone=0,Velocity_Z_Drone=0,Altitude_Drone=0;
    public double Roll_Drone=0,Pitch_Drone=0,Yaw_Drone=0;
    int flag_check_virtual_joystick_state=0;
    BaseProduct product = DJISDKManager.getInstance().getProduct();


    Publisher<std_msgs.Int32> Responses;
    Publisher<std_msgs.String> Responses_Debug;
    Subscriber<std_msgs.String> Receive_Command;
    Subscriber<geometry_msgs.Quaternion> Receive_Velocity;
    ConnectedNode Node;


    public Ros_Publishers() {
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("ROS_DJI/Ros_Publishers");
    }


    public void onStart(ConnectedNode connectedNode)
    {
        Node=connectedNode;

        Responses = connectedNode.newPublisher("Responses", "std_msgs/Int32");

        Responses_Debug = connectedNode.newPublisher("Responses_Debug", "std_msgs/String");

        Receive_Command = connectedNode.newSubscriber("Mavic_Commands", "std_msgs/String");

        Receive_Command.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                java.lang.String Command=message.getData();
                Parse_Command(Command);
            }
        });

        Receive_Velocity = connectedNode.newSubscriber("Velocity_Commands","geometry_msgs/Quaternion");
        Receive_Velocity.addMessageListener(new MessageListener<geometry_msgs.Quaternion>() {
            @Override
            public void onNewMessage(geometry_msgs.Quaternion message) {
                if(flag_check_virtual_joystick_state==1)
                {
                    Send_Flight_Control_Data_To_Aircraft((float) message.getX(),(float) message.getY(),(float) message.getZ(),(float)message.getW());
                }
            }
        });

        final Publisher<geometry_msgs.QuaternionStamped> Velocity = connectedNode.newPublisher("Velocity", "geometry_msgs/QuaternionStamped");
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            protected void loop() throws InterruptedException {
                Get_Velocity_Altitude();
                geometry_msgs.QuaternionStamped Value = (geometry_msgs.QuaternionStamped)Velocity.newMessage();
                geometry_msgs.Quaternion Value_Point = Value.getQuaternion();
                std_msgs.Header Head = Value.getHeader();
                Head.setStamp(connectedNode.getCurrentTime());
                Value_Point.setX(Velocity_X_Drone);
                Value_Point.setY(Velocity_Y_Drone);
                Value_Point.setZ(Velocity_Z_Drone);
                Value_Point.setW(Altitude_Drone);
                Value.setQuaternion(Value_Point);
                Value.setHeader(Head);
                Velocity.publish(Value);
                Thread.sleep(50L);
            }
        });

        final Publisher<geometry_msgs.PointStamped> Gimbal_Attitude = connectedNode.newPublisher("Gimbal_Attitude", "geometry_msgs/PointStamped");
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            protected void loop() throws InterruptedException {
                geometry_msgs.PointStamped Value = (geometry_msgs.PointStamped)Gimbal_Attitude.newMessage();
                geometry_msgs.Point Value_Point = Value.getPoint();
                std_msgs.Header Head = Value.getHeader();
                Head.setStamp(connectedNode.getCurrentTime());
                Value_Point.setX(Gimbal_Roll);
                Value_Point.setY(Gimbal_Pitch);
                Value_Point.setZ(Gimbal_Yaw);
                Value.setPoint(Value_Point);
                Value.setHeader(Head);
                Gimbal_Attitude.publish(Value);
                Thread.sleep(50L);
            }
        });

        final Publisher<geometry_msgs.PointStamped> Attitude = connectedNode.newPublisher("Attitude_RPY", "geometry_msgs/PointStamped");
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            protected void loop() throws InterruptedException {
                Get_Aircraft_Attitude();
                geometry_msgs.PointStamped Value = (geometry_msgs.PointStamped)Attitude.newMessage();
                geometry_msgs.Point Value_Point = Value.getPoint();
                std_msgs.Header Head = Value.getHeader();
                Head.setStamp(connectedNode.getCurrentTime());
                Value_Point.setX(Roll_Drone);
                Value_Point.setY(Pitch_Drone);
                Value_Point.setZ(Yaw_Drone);
                Value.setPoint(Value_Point);
                Value.setHeader(Head);
                Attitude.publish(Value);
                Thread.sleep(50L);
            }
        });

        Start_Receiving_Gimbal_Attitude();
        Start_Receiving_DJI_Data();
    }

    private void Start_Receiving_Gimbal_Attitude(){
        product.getGimbal().setStateCallback(new GimbalState.Callback(){
            @Override
            public void onUpdate(@NonNull GimbalState gimbalState) {
                Gimbal_Roll=gimbalState.getAttitudeInDegrees().getRoll();
                Gimbal_Pitch=gimbalState.getAttitudeInDegrees().getPitch();
                Gimbal_Yaw=gimbalState.getAttitudeInDegrees().getYaw();
            }
        });
    }

    public void Start_Receiving_DJI_Data()
    {

        DJIKey IsLandingConfirmation_key = FlightControllerKey.create(FlightControllerKey.IS_LANDING_CONFIRMATION_NEEDED);
        DJISDKManager.getInstance().getKeyManager().addListener(IsLandingConfirmation_key, new KeyListener() {
            @Override
            public void onValueChange(@Nullable Object oldValue, @Nullable Object newValue) {
                if(newValue instanceof Boolean)
                {
                    if((Boolean) newValue==true)
                    {
                        Confirm_Landing();
                    }
                }
            }
        });

    }

    private void Get_Aircraft_Attitude()
    {
        FlightController flightController = ((Aircraft) product).getFlightController();
        Roll_Drone = flightController.getState().getAttitude().roll;
        Pitch_Drone = flightController.getState().getAttitude().pitch;
        Yaw_Drone = flightController.getState().getAttitude().yaw;
    }
    private void Get_Velocity_Altitude(){
        FlightController flightController = ((Aircraft) product).getFlightController();
        Velocity_X_Drone=flightController.getState().getVelocityX();
        Velocity_Y_Drone=flightController.getState().getVelocityY();
        Velocity_Z_Drone=flightController.getState().getVelocityZ();
        Altitude_Drone=flightController.getState().getAircraftLocation().getAltitude();
    }

    public void Send_Debug_Message(java.lang.String message)
    {
        std_msgs.String String = (std_msgs.String)Responses_Debug.newMessage();
        String.setData(message);
        Responses_Debug.publish(String);
    }

    public void Send_Command_Result(int result)
    {
        std_msgs.Int32 Result = (std_msgs.Int32)Responses.newMessage();
        Result.setData(result);
        Responses.publish(Result);
    }

    public void Parse_Command(java.lang.String Command)
    {

        if(Command.equals("Motor Off"))
        {
            Motors(false);
        }
        else if(Command.equals("Motor On"))
        {
            Motors(true);
        }
        else if(Command.equals("Takeoff"))
        {
            TakeOff_Landing(false);
        }
        else if(Command.equals("Land"))
        {
            TakeOff_Landing(true);
        }
        else if(Command.equals("Joystick Off"))
        {
            Activate_Virtual_Joystick(false);
        }
        else if(Command.equals("Joystick On"))
        {
            Activate_Virtual_Joystick(true);
        }
        else if(Command.substring(0,4).equals("SetUp"))
        {
            Set_Up(Command.charAt(6),Command.charAt(8),Command.charAt(10),Command.charAt(12));
        }
        else
        {
            Send_Command_Result(0);
            Send_Debug_Message("Invalid Command");
        }
    }

    private void Send_Flight_Control_Data_To_Aircraft(float Velocity_X,float Velocity_Y,float Velocity_Z,float Velocity_Yaw)
    {
        if(Velocity_X>15)
        {
            Velocity_X=15;
        }
        else if (Velocity_X<-15)
        {
            Velocity_X=-15;
        }
        if(Velocity_Y>15)
        {
            Velocity_Y=15;
        }
        else if (Velocity_Y<-15)
        {
            Velocity_Y=-15;
        }
        if(Velocity_Z>4)
        {
            Velocity_Z=4;
        }
        else if (Velocity_Z<-4)
        {
            Velocity_Z=-4;
        }
        if(Velocity_Yaw>100)
        {
            Velocity_Yaw=100;
        }
        else if (Velocity_Yaw<-100)
        {
            Velocity_Yaw=-100;
        }
        FlightControlData velocity_data=null;
        velocity_data = new FlightControlData((float)Velocity_X,(float)Velocity_Y,(float)Velocity_Yaw,(float)Velocity_Z);
        ((Aircraft) product).getFlightController().sendVirtualStickFlightControlData(velocity_data, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if (djiError==null)
                {
                    Send_Command_Result(1);
                    Send_Debug_Message("Velocity Received");
                }
                else
                {
                    Send_Command_Result(0);
                    Send_Debug_Message("Velocity Error");
                }

            }
        });

    }

    private void Activate_Virtual_Joystick(boolean OnOff)
    {
        FlightController flightController = ((Aircraft) product).getFlightController();
        if(OnOff==false)
        {

                flightController.getVirtualStickModeEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
                    @Override
                    public void onSuccess(Boolean aBoolean)
                    {
                        if(aBoolean.booleanValue()==true)
                        {
                            flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    if(djiError!=null)
                                    {
                                        Send_Command_Result(0);
                                        Send_Debug_Message("Virtual Joystick Deactivation Error");
                                    }
                                    else
                                    {
                                        flightController.getVirtualStickModeEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
                                            @Override
                                            public void onSuccess(Boolean aBoolean) {
                                                if(aBoolean.booleanValue()==false)
                                                {
                                                    Send_Command_Result(1);
                                                    Send_Debug_Message("Virtual Joystick Deactivated");
                                                    flag_check_virtual_joystick_state=0;
                                                }
                                            }

                                            @Override
                                            public void onFailure(DJIError djiError) {

                                            }
                                        });
                                    }
                                }
                            });
                        }
                    }
                    @Override
                    public void onFailure(DJIError djiError) {

                    }
                });
        }
        else if(OnOff==true)
        {
            flightController.getVirtualStickModeEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
                @Override
                public void onSuccess(Boolean aBoolean) {
                    if(aBoolean.booleanValue()==false)
                    {
                        flightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                if (djiError != null) {
                                    Send_Command_Result(0);
                                    Send_Debug_Message("Virtual Joystick Activation Error");
                                }
                                else {
                                    flightController.getVirtualStickModeEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
                                        @Override
                                        public void onSuccess(Boolean aBoolean) {
                                            if(aBoolean.booleanValue()==true)
                                            {
                                                Send_Command_Result(1);
                                                Send_Debug_Message("Virtual Joystick Activated");
                                                flag_check_virtual_joystick_state = 1;
                                            }
                                        }

                                        @Override
                                        public void onFailure(DJIError djiError) {

                                        }
                                    });
                                }

                            }
                        });
                    }
                }
                @Override
                public void onFailure(DJIError djiError) {

                }
            });
        }
    }

    private void Set_Up(char Vertical, char RollPitchControl, char YawControl, char CoordinateSystem)
    {
        Set_Up_Vertical(Vertical);
        Set_Up_Roll_Pitch(RollPitchControl);
        Set_Up_Yaw_Control(YawControl);
        Set_Up_Coordinate_System(CoordinateSystem);
    }

    private void Motors(boolean OnOff)
    {
        DJIKey Motor_Key=null;
        if (OnOff == false)
        {
            Motor_Key = FlightControllerKey.create(FlightControllerKey.TURN_OFF_MOTORS);
            KeyManager.getInstance().performAction(Motor_Key, new ActionCallback() {
                @Override
                public void onSuccess()
                {
                    Send_Command_Result(1);
                    Send_Debug_Message("Stop Motors Success");
                }

                @Override
                public void onFailure(@NonNull DJIError djiError)
                {
                    Send_Command_Result(0);
                    Send_Debug_Message(djiError.getDescription());
                }
            });
        }
        else if(OnOff == true)
        {
            Motor_Key = FlightControllerKey.create(FlightControllerKey.TURN_ON_MOTORS);
            KeyManager.getInstance().performAction(Motor_Key, new ActionCallback() {
                @Override
                public void onSuccess()
                {
                    Send_Command_Result(1);
                    Send_Debug_Message("Start Motors Success");
                }

                @Override
                public void onFailure(@NonNull DJIError djiError)
                {
                    Send_Command_Result(0);
                    Send_Debug_Message(djiError.getDescription());
                }
            });

        }
    }

    private void TakeOff_Landing(boolean TakeoffLanding)
    {
        DJIKey TakeOff_Landing_Key=null;
        if (TakeoffLanding == false)
        {
                TakeOff_Landing_Key = FlightControllerKey.create(FlightControllerKey.TAKE_OFF);
                KeyManager.getInstance().performAction(TakeOff_Landing_Key, new ActionCallback() {
                    @Override
                    public void onSuccess()
                    {
                        Send_Command_Result(1);
                        Send_Debug_Message("Take Off Started");
                    }
                    @Override
                    public void onFailure(@NonNull DJIError djiError)
                    {
                        Send_Debug_Message(djiError.getDescription());
                        Send_Command_Result(0);
                    }
                });
        }
        else if(TakeoffLanding == true)
        {
                TakeOff_Landing_Key = FlightControllerKey.create(FlightControllerKey.START_LANDING);
                KeyManager.getInstance().performAction(TakeOff_Landing_Key, new ActionCallback() {
                    @Override
                    public void onSuccess()
                    {
                        Send_Command_Result(1);
                        Send_Debug_Message("Landing Started");
                    }

                    @Override
                    public void onFailure(@NonNull DJIError djiError)
                    {
                        Send_Command_Result(0);
                        Send_Debug_Message(djiError.getDescription());
                    }
                });
        }
    }

    private void Confirm_Landing()
    {
        DJIKey TakeOff_Landing_Key=null;
        TakeOff_Landing_Key = FlightControllerKey.create(FlightControllerKey.CONFIRM_LANDING);
        KeyManager.getInstance().performAction(TakeOff_Landing_Key, new ActionCallback() {
            @Override
            public void onSuccess()
            {

            }

            @Override
            public void onFailure(@NonNull DJIError djiError)
            {

            }
        });
    }

    private void Set_Up_Vertical(char Vertical)
    {
        VerticalControlMode vertical_mode=VerticalControlMode.VELOCITY;
        int i=-1;
        java.lang.String s="";

        if(Vertical==0x30)
        {
            vertical_mode = VerticalControlMode.VELOCITY;
            i=0;
            s="Vertical Control Mode = Velocity";
        }
        else if(Vertical==0x31)
        {
            vertical_mode = VerticalControlMode.POSITION;
            i=1;
            s="Vertical Control Mode = Position";
        }
        ((Aircraft) product).getFlightController().setVerticalControlMode(vertical_mode);
        VerticalControlMode Status=((Aircraft) product).getFlightController().getVerticalControlMode();
        if(Status.value()==i)
        {
            Send_Command_Result(1);
            Send_Debug_Message(s);
        }
        else
        {
            Send_Command_Result(1);
            Send_Debug_Message("Vertical Control Mode Setup Error");
        }
    }

    private void Set_Up_Roll_Pitch(char RollPitch)
    {
        RollPitchControlMode RollPitch_mode=RollPitchControlMode.VELOCITY;
        int i=-1;
        java.lang.String s="";

        if(RollPitch==0x30)
        {
            RollPitch_mode = RollPitchControlMode.ANGLE;
            s="Roll Pitch Control Mode = Angle";
            i=0;
        }
        else
        {
            RollPitch_mode = RollPitchControlMode.VELOCITY;
            s="Roll Pitch Control Mode = Velocity";
            i=1;
        }
        ((Aircraft) product).getFlightController().setRollPitchControlMode(RollPitch_mode);
        RollPitchControlMode Status=((Aircraft) product).getFlightController().getRollPitchControlMode();
        if(Status.value()==i)
        {
            Send_Command_Result(1);
            Send_Debug_Message(s);
        }
        else
        {
            Send_Command_Result(1);
            Send_Debug_Message("Roll Pitch Control Mode Setup Error");
        }
    }

    private void Set_Up_Yaw_Control(char YawControl)
    {
        int i=-1;
        java.lang.String s="";
        YawControlMode Yaw_mode=YawControlMode.ANGULAR_VELOCITY;
        if(YawControl==0x30)
        {
            Yaw_mode = YawControlMode.ANGLE;
            s="Yaw Control Mode = Angle";
            i=0;
        }
        else
        {
            Yaw_mode = YawControlMode.ANGULAR_VELOCITY;
            s="Yaw Control Mode = Angular Velocity";
            i=1;
        }

        ((Aircraft) product).getFlightController().setYawControlMode(Yaw_mode);
        YawControlMode Status=((Aircraft) product).getFlightController().getYawControlMode();
        if(Status.value()==i)
        {
            Send_Command_Result(1);
            Send_Debug_Message(s);
        }
        else
        {
            Send_Command_Result(1);
            Send_Debug_Message("Yaw Control Mode Setup Error");
        }
    }

    private void Set_Up_Coordinate_System(char CoordinateSystem)
    {
        int i=-1;
        java.lang.String s="";
        FlightCoordinateSystem System_mode=FlightCoordinateSystem.BODY;
        if(CoordinateSystem==0x30)
        {
            System_mode = FlightCoordinateSystem.GROUND;
            s="Flight Coordinate System = Ground";
            i=0;
        }
        else
        {
            System_mode = FlightCoordinateSystem.BODY;
            s="Flight Coordinate System = Body";
            i=1;
        }
        ((Aircraft) product).getFlightController().setRollPitchCoordinateSystem(System_mode);
        FlightCoordinateSystem Status=((Aircraft) product).getFlightController().getRollPitchCoordinateSystem();
        if(Status.value()==i)
        {
            Send_Command_Result(1);
            Send_Debug_Message(s);
        }
        else
        {
            Send_Command_Result(1);
            Send_Debug_Message("Flight Coordinate System Setup Error");
        }
    }
}
