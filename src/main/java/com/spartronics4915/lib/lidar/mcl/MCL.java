package com.spartronics4915.lib.lidar.mcl;

import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Twist2;
import com.spartronics4915.lib.geometry.Rotation2;
import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.lidar.LidarScan;

/**
 * Monte Carlo Localization
 * 
 */
public class MCL
{
    private static double spare;
    private static boolean isSpareReady = false;

    private final static int k_particleCount = 100;
    private final static double k_deltaDegrees = 10; // +- 5 degrees noise
    private final static double k_deltaDist = 4; // +- 4 inches
    private Map m_map;
    private Particle[] m_particles;
    private Pose2 m_initialPose;

    public MCL(Pose2 initialPose, Map map)
    {
        m_initialPose = initialPose;
        m_map = map;
        initParticles();
    }

    /**
     * 
     * @param deltaPose
     * @param scan
     * @return new estimated Pose2
     */
    public Pose2 update(Twist2 deltaPose, LidarScan scan)
    {
        Pose2 nextPose;

        return nextPose;
    }

    /**
     * Given an initial Pose2, initialize our particle filter with
     * a number of variant poses according to a gaussian distribution.
     */
    private void initParticles()
    {
        m_particles = new Particle[k_particleCount];
        moveParticles(m_initialPose, new Twist2());
    }

    private void moveParticles(Pose2 initPose, Twist2 deltaPose)
    {
        Rotation2 initHeading = initPose.getRotation();
        Translation2 initPos = initPose.getTranslation();
        for(Particle p : m_particles)
        {
            double deg = randomGaussian(0, k_deltaDegrees);
            Rotation2 r = initHeading.rotateBy(Rotation2.fromDegrees(deg));
            Translation2 t = initPos.translateBy(  
                                randomGaussian(0, k_deltaDist),
                                randomGaussian(0, k_deltaDist));
            p.pose = new Pose2(t, r);
            p.weight = 1.0; // XXX
        }
    }

    /**
     * 
     * marsaglia's method for generating gaussian-distributed numbers
     */
    private double randomGaussian(double mean, double stdDev)
    {
        if (isSpareReady) 
        {
            isSpareReady = false;
            return spare * stdDev + mean;
        } 
        else 
        {
            double u, v, s;
            do 
            {
                u = Math.random() * 2 - 1;
                v = Math.random() * 2 - 1;
                s = u * u + v * v;
            } while (s >= 1 || s == 0);
            double mul = Math.sqrt(-2.0 * Math.log(s) / s);
            spare = v * mul;
            isSpareReady = true;
            return mean + stdDev * u * mul;
        }
    }
}   