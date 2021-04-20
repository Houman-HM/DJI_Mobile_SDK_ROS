package com.dji.ros_dji;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.MessageBuffers;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.io.IOException;

import sensor_msgs.CompressedImage;

public class Camera_Publisher extends AbstractNodeMain {


    public Camera_Publisher() {
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("ROS_DJI/Camera_Publisher");
    }


    public void onStart(ConnectedNode connectedNode) {
        final Publisher<CompressedImage> Video = connectedNode.newPublisher("Video_Feed", "sensor_msgs/CompressedImage");
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            protected void loop() throws InterruptedException {
                if (Camera_Node.flag_go==1) {
                    CompressedImage Value = (CompressedImage) Video.newMessage();
                    ChannelBufferOutputStream stream_1 = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
                    try {

                        stream_1.write(Camera_Node.byteArray);

                    } catch (IOException e) {
                        e.printStackTrace();
                    }

                    Value.setData(stream_1.buffer().copy());
                    stream_1.buffer().clear();
                    Video.publish(Value);
                }
                    Thread.sleep(16L);
            }
        });


    }


}
