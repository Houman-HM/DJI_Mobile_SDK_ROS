package com.dji.ros_dji;


import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MainActivity extends Activity {

    ROS_Connection Connect_Ros;
    public MainActivity() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
    }

    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        //Sensor_Data_Func.Start_CallBacks();
        Intent intent = new Intent(this, ROS_Connection.class);
        startActivity(intent);
    }

}

