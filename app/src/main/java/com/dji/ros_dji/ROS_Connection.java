package com.dji.ros_dji;

import android.content.Intent;
import android.os.Bundle;

import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

public class ROS_Connection extends RosActivity {

    private Ros_Publishers Pub_Node;
    private Camera_Publisher Cam_Node;

    public ROS_Connection() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("ROS_DJI", "ROS_DJI", URI.create("http://192.168.43.171:11311"));
    }

    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        //Sensor_Data_Func.Start_CallBacks();
        //setContentView(R.layout.main);
    }

    @Override
    public void init(NodeMainExecutor nodeMainExecutor) {
        Pub_Node=new Ros_Publishers();
        Cam_Node=new Camera_Publisher();
        // At this point, the user has already been prompted to either enter the URI
        // of a master to use or to start a master locally.

        // The user can easily use the selected ROS Hostname in the master chooser
        // activity.
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(Pub_Node, nodeConfiguration);
        nodeMainExecutor.execute(Cam_Node, nodeConfiguration);
        Intent intent = new Intent(this, Camera_Node.class);
        startActivity(intent);
        // The RosTextView is also a NodeMain that must be executed in order to
        // start displaying incoming messages.

    }
}