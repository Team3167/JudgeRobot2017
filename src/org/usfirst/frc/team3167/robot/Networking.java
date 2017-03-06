package org.usfirst.frc.team3167.robot;

import java.io.IOException;
import java.net.DatagramSocket;
import java.net.DatagramPacket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

public class Networking {
	
	public class RobotPosition {
	    public double x, y;// [inches]
	    public double theta;// [rad]
	}

    private final byte[] buffer = new byte[128];

    private final DatagramSocket socket;
    private final DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
    
    public Networking(int port) throws SocketException {
        socket = new DatagramSocket(port);
        socket.setSoTimeout(1);// block for no more than 1 msec
    }
    
    public boolean GotPositionUpdate() {
        try
        {
            socket.receive(packet);
            /*System.out.println("Received " + packet.getData().length + " bytes from " + packet.getAddress());
            System.out.println("Payload =");
            int i, j;
            for (j = 0; j < 3; j++)
            {
	            for (i = 0; i < 4; i++)
	            {
	            	System.out.printf("0x%02X ", packet.getData()[i]);
	            }
	            
	            System.out.println("");
            }//*/
        }
        catch (SocketTimeoutException e)
        {
            return false;
        }
        catch (IOException e)
        {
        	// TODO:  Anything else here - this one could really be a problem we want to know about?
        	return false;
        }
        
        return true;
    }
    
    public RobotPosition GetLatestPosition() {
        FloatBuffer dArray = ByteBuffer.wrap(packet.getData()).asFloatBuffer();
        RobotPosition robotPosition = new RobotPosition();
        robotPosition.x = dArray.get(0);
        robotPosition.y = dArray.get(1);
        robotPosition.theta = dArray.get(2);
        return robotPosition;
    }
    
    // This must be called immediately prior to beginning use of image data to
    // ensure we're not using old data.  For example, if we begin a "hang gear"
    // task after a button is pressed, we'd need to write something like:
    // if (buttonIsPressed) { net.FlushBuffer(); DoGearHangTask(); }
    // In subsequent frames, we do NOT call FlushBuffer, as there would then be
    // no data to process.
    public void FlushBuffer() {
        while (GotPositionUpdate()) {}
    }

}
