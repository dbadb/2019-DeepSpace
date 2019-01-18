package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.lib.util.RobotStateMap;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.geometry.Translation2;

public class WaitUntilInsideRegion implements Action
{

    private final static RobotStateMap mRobotStateMap = RobotStateEstimator.getInstance().getEncoderRobotStateMap();

    private final Translation2 mBottomLeft;
    private final Translation2 mTopRight;

    public WaitUntilInsideRegion(Translation2 bottomLeft, Translation2 topRight, boolean isOnLeft)
    {
        if (isOnLeft)
        {
            mBottomLeft = new Translation2(bottomLeft.x(), -topRight.y());
            mTopRight = new Translation2(topRight.x(), -bottomLeft.y());
        }
        else
        {
            mBottomLeft = bottomLeft;
            mTopRight = topRight;
        }
    }

    @Override
    public boolean isFinished()
    {
        Translation2 position = mRobotStateMap.getLatestFieldToVehicle().getTranslation();
        return position.x() > mBottomLeft.x() && position.x() < mTopRight.x()
                && position.y() > mBottomLeft.y() && position.y() < mTopRight.y();
    }

    @Override
    public void update()
    {

    }

    @Override
    public void done()
    {

    }

    @Override
    public void start()
    {

    }
}
