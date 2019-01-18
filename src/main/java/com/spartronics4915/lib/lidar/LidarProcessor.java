package com.spartronics4915.lib.lidar;

import com.spartronics4915.lib.LibConstants;

import com.spartronics4915.lib.lidar.icp.ICP;

import com.spartronics4915.lib.lidar.icp.IReferenceModel;
import com.spartronics4915.lib.lidar.icp.RelativeICPProcessor;
import com.spartronics4915.lib.geometry.Transform2;

import com.spartronics4915.lib.util.ILoop;

import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.geometry.Twist2;
import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Point2;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.RobotStateMap;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.DoubleSupplier;
import java.util.concurrent.LinkedBlockingQueue;

import java.util.LinkedHashMap;
import java.util.Map;
import java.net.URI;
import java.net.URISyntaxException;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

/**
 * Receives LIDAR points from the {@link LidarServer}, stores a set number of
 * scans/revolutions, and provides methods for processing the data.
 * <p>
 * All interfacing with the LIDAR should be done through this class.
 *
 * @see Constants.kLidarNumScansToStore
 * @see doICP()
 * @see getTowerPosition()
 */

class WSClient extends WebSocketClient
{
    private boolean isOpen;
    public WSClient() throws URISyntaxException
    {
        //super(new URI("ws://192.168.1.10:5080/webapi/_publish_"));
        super(new URI("ws://localhost:5080/webapi/_publish_"));
        isOpen = false;
    }

    @Override
    public void onOpen(ServerHandshake handshakedata)
    {
        Logger.info("WebSocket open");
        isOpen = true;
    }

    @Override
    public void onClose(int code,  String reason, boolean remote)
    {
        Logger.info("WebSocket close");
        isOpen = false;
    }

    @Override
    public void onError(Exception ex)
    {
        Logger.exception(ex);
    }

    @Override
    public void onMessage(String message)
    { // Logger.info("WebSocket message "  + message);
    }

    public void send(LidarScan scan)
    {
        if(isOpen)
        {
            this.send(scan.toJsonString());
        }
    }
}

public class LidarProcessor implements ILoop 
{
    public enum RunMode
    {
        kRunInRobot,
        kRunAsTest
    };

    enum OperatingMode
    {
        kRelativeICP,
        kAbsoluteICP,
        kMCL
    };

    private static boolean sDebugPoints = false;
    private final Pose2 kVehicleToLidar;

    private LidarServer mLidarServer;
    private double mScanTime;
    private double mLastScanTime;
    private double mScanTimeAccum;
    private int mScanCount;
    private ICP mICP; 
    private RelativeICPProcessor mRelativeICP; 
    private final ReadWriteLock mRWLock; 
    private LinkedBlockingQueue<LidarScan> mScanQueue;
    private LidarScan mActiveScan;
    private final OperatingMode mMode = OperatingMode.kRelativeICP;
    private WSClient mWSClient;
    private IReferenceModel mReferenceModel;
    private RobotStateMap mEncoderStateMap;
    private RobotStateMap mLidarStateMap;
    private DoubleSupplier mTimeSupplier;

    // A scan is a collection of lidar points.  Each point has an associated
    // timestamp and the scan's timestamp is that of its first point.
    // Currently, chezy_lidar delivers ~15 pt clumps with the same timestamp.
    // If a scan is 360 points, then we expect 24 (360/15) clumps per scan.
    // Since a full scan requires 1/8hz seconds each clump represents
    // .005 sec (1/(24*8)) timeslices.  Since these timestamps are processed
    // in order we only need to keep the last-most Pose2.
    private double mLastPoseTimestamp;
    private Pose2 mLastRobotPose = null;

    public LidarProcessor(RunMode runMode, IReferenceModel refmodel,
        RobotStateMap encoderStateMap, RobotStateMap lidarStateMap, 
        Pose2 vehicleToLidar, DoubleSupplier timeSupplier) 
    {
        Logger.debug("LidarProcessor starting...");
        kVehicleToLidar = vehicleToLidar;

        mICP = new ICP(LibConstants.kICPTimeoutMs);
        mScanQueue = new LinkedBlockingQueue<LidarScan>();
        mRelativeICP = new RelativeICPProcessor(mICP);
        mRWLock = new ReentrantReadWriteLock();
        mLidarServer = new LidarServer(this, timeSupplier);
        mScanTime = Double.NEGATIVE_INFINITY;
        mLastScanTime = Double.NEGATIVE_INFINITY;
        mScanTimeAccum = 0;
        mScanCount = 0;
        mActiveScan = null;
        mReferenceModel = refmodel; // may be null
        mEncoderStateMap = encoderStateMap;
        mLidarStateMap = lidarStateMap; // This could be the same object as above
        mTimeSupplier = timeSupplier;
        mLastPoseTimestamp = 0;

        if(runMode == RunMode.kRunAsTest)
        {
            try 
            {
                mWSClient = new WSClient();
                mWSClient.connect();
            }
            catch(URISyntaxException e)
            {
                Logger.exception(e);
            }
        }
    }

    public boolean isConnected()
    {
        return mLidarServer.isLidarConnected();
    }

    @Override
    public void onStart(double timestamp) 
    {
    }

    @Override
    public void onLoop(double timestamp) 
    {
        // we're called regularly (100hz) from the looper. 
        if (timestamp - getScanStart() > LibConstants.kLidarRestartTime) 
        {
            if (!mLidarServer.isEnding() && !mLidarServer.isRunning() &&
                mLidarServer.isLidarConnected())  
            {
                if(!mLidarServer.start())   
                {
                    // If server fails to start we update mScanTime to 
                    // ensure we don't restart each loop.
                    startNewScan(timestamp);
                }
            }
        }
        if(mLidarServer.isRunning())
        {
            try
            {
                LidarScan scan = mScanQueue.take(); // consumer blocks
                double scanTime = scan.getTimestamp();
                if(mScanCount > 0)
                    mScanTimeAccum += scanTime - mLastScanTime;
                if(mScanCount%10 == 1)
                {
                    double scansPerSec = mScanCount/mScanTimeAccum;
                    // we might want to log this to SmartDashboard
                    Logger.notice("scan " + mScanCount + 
                                  " npts:" + scan.getScanSize() +
                                  " scansPerSec:"+ scansPerSec);
                }
                mScanCount++;
                mLastScanTime = scanTime;
                this.processLidarScan(scan);
                if(mWSClient != null)
                    mWSClient.send(scan);
            }
            catch(InterruptedException ie)
            {
            }
        }
    }

    @Override
    public void onStop(double timestamp)
    {
        mLidarServer.stop();
    }

    private void processLidarScan(LidarScan scan)
    {
        try
        {
            final RobotStateMap.State lastState = mLidarStateMap.getLatestState();
            final Pose2 lastPose = lastState.pose;
            double dt = scan.getTimestamp() - lastState.timestamp;
            Pose2 poseEstimate;
            Twist2 velPredicted, velMeasured;
            if(mMode == OperatingMode.kRelativeICP) // no reliance on Encoder
            {
                Transform2 xform = mRelativeICP.doRelativeICP(scan);
                if(xform != null)
                {
                    // XXX: is xform.inverse correct?
                    Twist2 fwdK = Pose2.log(xform.inverse().toPose2());
                    // fwdK represents the change between scans
                    // we can directly "add" this to the last post estimate.
                    poseEstimate = lastPose.transformBy(Pose2.exp(fwdK));

                    // in (or radians) / time between scans -> in (or radians) / seconds
                    // Assumes that timestamps are in seconds
                    velMeasured = fwdK;

                    velPredicted = new Twist2(velMeasured.dx / dt,
                                               velMeasured.dy / dt,
                                               velMeasured.dtheta / dt);
                }
                else
                {
                    Logger.warning("Relative ICP returned a null transform!");
                    return;
                }
            } 
            else
            {
                // XXX: this needs validation
                // XXX: need to invoke getFieldToLidar, not getFieldToVehicle
                Transform2 xform = mICP.doICP(scan, 
                                new Transform2(lastPose).inverse(),  // ie: LidarToField
                                mReferenceModel); // mReferenceMode in field coords
                Twist2 fwdK = Pose2.log(xform.inverse().toPose2());
                poseEstimate = lastPose.transformBy(Pose2.exp(fwdK));
                velMeasured = fwdK;
                velPredicted = new Twist2(velMeasured.dx / dt,
                                            velMeasured.dy / dt,
                                            velMeasured.dtheta / dt);
            }
            mLidarStateMap.addObservations(scan.getTimestamp(), 
                                poseEstimate, velMeasured, velPredicted);
        }
        catch(Exception e)
        {
            Logger.exception(e);
        }
    }

    // addSample is invoked from LidarServer::handleLine via the ReaderThread.
    // logging is only invoked from this thread, but scan data is accessed
    // asynchronously from the main thread (which, for example, performs
    // ICP matching). This accounts for the use of our read-write lock on the
    // scan data.  Any changes to mScans or an individual scan should
    // be guarded by this lock.
    public void addSample(double ts, double angle, double dist, boolean newScan) 
    {
        // transform by the robot's pose
        if(ts != mLastPoseTimestamp)
        {
            mLastPoseTimestamp = ts;
            mLastRobotPose = mEncoderStateMap.getFieldToVehicle(ts).transformBy(kVehicleToLidar);
        }
        LidarSample lpt = new LidarSample(ts, angle, dist); 
        if (newScan || mActiveScan == null) 
        { 
            if(mActiveScan != null)
                mScanQueue.add(mActiveScan); // <- send last scan to consumer
            mActiveScan  = new LidarScan();
            startNewScan(mTimeSupplier.getAsDouble());
        }
        mActiveScan.addSample(lpt, mLastRobotPose);
    }

    public void startNewScan(double time) 
    {
        mRWLock.writeLock().lock();
        try 
        {
            mScanTime = time;
        } 
        finally 
        {
            mRWLock.writeLock().unlock();
        }
    }

    public double getScanStart() 
    {
        mRWLock.readLock().lock();
        try 
        {
            return mScanTime;
        } 
        finally 
        {
            mRWLock.readLock().unlock();
        }
    }
}
