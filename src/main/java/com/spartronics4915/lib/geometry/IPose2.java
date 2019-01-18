package com.spartronics4915.lib.geometry;

public interface IPose2<S> extends IRotation2<S>, ITranslation2<S>
{

    public Pose2 getPose();

    public S transformBy(Pose2 transform);

    public S mirror();
}
