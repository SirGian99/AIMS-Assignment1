using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class NewNode{
    public static HashSet<NewNode> nodes = new HashSet<NewNode>();

    public bool duplicate;
    public Vector4 continous_position;
    public Vector3 xyz_position
    {
        get
        {
            return new Vector3(continous_position.x, 0, continous_position.z);
        }
    }

    public Vector3 discrete_xyz_position
    {
        get
        {
            return new Vector3(discrete_position.x, 0, discrete_position.z);
        }
    }
    public Vector4 discrete_position
    {
        get
        {
            return new Vector4((int)Math.Round(continous_position.x), (int)Math.Round(continous_position.y), (int)Math.Round(continous_position.z), (int)Math.Round(continous_position.w));
        }
    }
    public float gCost; //distance from the starting node
    public float hCost; //distance from the arrival node
    public float additional_cost; //additional hybrid cost
    public float fCost
    {
        get
        {
            //Debug.Log("Node [" + i + "," + j + "] penalty " + wallClosenessCost +" fcost: " + (gCost + hCost + hybridAdditionalCost + wallClosenessCost));
            //TODO UNCOMMENT return gCost + hCost + additional_cost;
            return gCost + hCost;
        }
    }

    public NewNode parent;
    public NewNode(Vector4 position)
    {
        continous_position = position;
        duplicate = !nodes.Add(this);
    }

    public override bool Equals(object obj)
    {
        return discrete_position == ((NewNode)obj).discrete_position;
    }

    public bool EqualsContinous(NewNode obj)
    {
        return continous_position == obj.continous_position;
    }


    public class NodeComp : IComparer<NewNode>
    {
        public int Compare(NewNode x, NewNode y)
        {
            int f_difference = (int)(x.fCost - y.fCost);
            if (f_difference == 0)
                return (int)(x.hCost - y.hCost);
            return f_difference;
        }

    }

    public override int GetHashCode()
    {
        int hash = 13;
        hash = (hash * 7) + discrete_position.GetHashCode();
        return hash;
    }

    public static NewNode FindNode(Vector4 position) //if no node is found a new one is created
    {
        NewNode toFind = new NewNode(position);
        if (toFind.duplicate)
        {
            foreach(NewNode node in nodes)
            {
                if (node.Equals(toFind))
                    return node;
            }
        }
        return toFind;

    }

    public bool isInGoalRange(Vector3 position, float radius)
    {
        bool result =  Math.Abs(position.x - continous_position.x) <= radius && Math.Abs(position.z - continous_position.z) <= radius;
        if (result)
            Debug.Log("IN RANGE");
        return result;
    }
}
