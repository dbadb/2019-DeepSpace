package com.spartronics4915.lib.lidar.mcl;
import com.spartronics4915.lib.geometry.Pose2;

// kd tree is used by Mixture/Dual-MCL. currently WIP
public class KDTree
{
    class Data
    {
        public double[] data; // length 
    }

    class Node
    {
        public int splitVar;
        public double splitVal;
        public Node left;
        public Node right;
        public double[] data; // length: sizeof(Pose2) + numSensors
    }

    static Node buildTree(int firstVar, int maxVar, int numData, double[] data)

    {
        Node ret = null;
        return ret;
    }
    
    private Node[] m_nodes;

    KDTree()
    {

    }
}