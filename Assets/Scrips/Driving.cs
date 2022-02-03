using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DubinsPath // OneDubinsPath
{
    public float totalLength;

    public float length1;
    public float length2;
    public float length3;

    public Vector3 tangent1;
    public Vector3 tangent2;

    public GenerateDrivingDirections.PathType pathType;

    public List<Vector3> pathCoords;

    public bool segment2Turning;

    public bool segment1TurningRight;
    public bool segment2TurningRight;
    public bool segment3TurningRight;

    public DubinsPath(float length1, float length2, float length3, Vector3 tangent1, Vector3 tangent2, GenerateDrivingDirections.PathType pathType)
    {
        this.totalLength = length1 + length2 + length3;

        this.length1 = length1;
        this.length2 = length2;
        this.length3 = length3;

        this.tangent1 = tangent1;
        this.tangent2 = tangent2;

        this.pathType = pathType;

        DubinsMath dubinsMath = new DubinsMath();
    }

    public void SetIfTurningRight(bool segment1TurningRight, bool segment2TurningRight, bool segment3TurningRight)
    {
        this.segment1TurningRight = segment1TurningRight;
        this.segment2TurningRight = segment2TurningRight;
        this.segment3TurningRight = segment3TurningRight;
    }

}
public class GenerateDrivingDirections // Dubins
{
    // This is based on Dubins
    public enum PathType { LSL, LSR, RSR, RSL, LRL, RLR }

    public Vector3 sLeft;
    public Vector3 sRight;
    public Vector3 gLeft;
    public Vector3 gRight;

    public UnityStandardAssets.Vehicles.Car.CarController m_Car;

    // And where are we now? Position and location

    Vector3 startPos;
    Vector3 goalPos;

    float startHeading;
    float goalHeading;

    // And finally, how fast are we going?

    float currentVelocity;

    // Let's make a list of the possible paths
    List<DubinsPath> pathList = new List<DubinsPath>();

    public GenerateDrivingDirections(UnityStandardAssets.Vehicles.Car.CarController m_Car)
    {
        this.m_Car = m_Car;
    }

    public List<DubinsPath> makeManyDubinsPaths(Vector3 startPos, float startHeading, Vector3 goalPos, float goalHeading)
    {
        //This does not take the current velocity into consideration right now
        this.startPos = startPos;
        this.goalPos = goalPos;
        this.startHeading = startHeading;
        this.goalHeading = goalHeading;

        //Let's start with an empty list
        pathList.Clear();

        //Let's place the circles, one left and one right of the vehicles
        PositionLeftRightCircles();

        //Find the length of the paths 
        CalculateDubinsPathsL();

        //Do we have paths?
        if (pathList.Count > 0)
        {
            // Which one is the shortest?
            pathList.Sort((x, y) => x.totalLength.CompareTo(y.totalLength));

            //Find the final coordinates of the path, using the tangent points and the lengths
            GeneratePathCoordinates(); //TODO 

            return pathList;
        }
        else
        {
            return null; //No paths could be found
        }
    }

    public static void PositionLeftRightCircles()
    {
        //Start pos
        sRight = DubinsMath.RightCircleCenter(startPos, startHeading);
        sLeft = DubinsMath.LeftCircleCenter(startPos, startHeading);
        //Goal 
        gRight = DubinsMath.RightCircleCenter(goalPos, goalHeading);
        gLeft = DubinsMath.LeftCircleCenter(goalPos, goalHeading);
    }

    // Calculate the lengths of the paths
    void CalculateDubinsPathsL()
    {
        //Check that the circles are not in the same locations

        //RSR
        if (sRight.x != gRight.x && sRight.z != gRight.z)
        {
            Get_RSR_Length();
        }

        //LSL
        if (sLeft.x != gLeft.x && sLeft.z != gLeft.z)
        {
            Get_LSL_Length();
        }

        //RSL and LSR work as long as the circles are non-intersecting
        //RSL
        float comparisonSqr = DubinsMath.turningRadius * 2f * DubinsMath.turningRadius * 2f;

        //RSL
        if ((sRight - gLeft).sqrMagnitude > comparisonSqr)
        {
            Get_RSL_Length();
        }
        //LSR
        if ((sLeft - gRight).sqrMagnitude > comparisonSqr)
        {
            Get_LSR_Length();
        }

        //For LRL and RLR, the distance between the circles must be < 4r
        comparisonSqr = DubinsMath.turningRadius * 4f * DubinsMath.turningRadius * 4f;

        // LRL
        if ((sLeft - gLeft).sqrMagnitude < comparisonSqr)
        {
            Get_LRL_Length();
        }
        //RLR
        if ((sRight - gRight).sqrMagnitude < comparisonSqr)
        {
            Get_RLR_Length();
        }
    }

    //RSR
    void Get_RSR_Length()
    {
        //Find both tangent positons
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;

        DubinsMath.LSLorRSR(sRight, gRight, false, out startTangent, out goalTangent);

        //Calculate lengths
        float length1 = DubinsMath.GetArcLength(sRight, startPos, startTangent, false);

        float length2 = (startTangent - goalTangent).magnitude;

        float length3 = DubinsMath.GetArcLength(gRight, goalTangent, goalPos, false);

        //Save the data
        DubinsPath pathData = new DubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RSR);

        //We also need this data to simplify when generating the final path
        pathData.segment2Turning = false;

        //RSR
        pathData.SetIfTurningRight(true, false, true);

        //Add the path to the collection of all paths
        pathList.Add(pathData);
    }

    //LSL
    void Get_LSL_Length()
    {
        //Find both tangent positions
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;

        DubinsMath.LSLorRSR(sLeft, gLeft, true, out startTangent, out goalTangent);

        //Calculate lengths
        float length1 = DubinsMath.GetArcLength(sLeft, startPos, startTangent, true);

        float length2 = (startTangent - goalTangent).magnitude;

        float length3 = DubinsMath.GetArcLength(gLeft, goalTangent, goalPos, true);

        //Save the data
        DubinsPath pathData = new DubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LSL);

        //We also need this data to simplify when generating the final path
        pathData.segment2Turning = false;

        //LSL
        pathData.SetIfTurningRight(false, false, false);

        //Add the path to the collection of all paths
        pathList.Add(pathData);
    }


    //RSL
    void Get_RSL_Length()
    {
        //Find both tangent positions
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;

        DubinsMath.RSLorLSR(sRight, gLeft, false, out startTangent, out goalTangent);

        //Calculate lengths
        float length1 = DubinsMath.GetArcLength(sRight, startPos, startTangent, false);

        float length2 = (startTangent - goalTangent).magnitude;

        float length3 = DubinsMath.GetArcLength(gLeft, goalTangent, goalPos, true);

        //Save the data
        DubinsPath pathData = new DubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RSL);

        //We also need this data to simplify when generating the final path
        pathData.segment2Turning = false;

        //RSL
        pathData.SetIfTurningRight(true, false, false);

        //Add the path to the collection of all paths
        pathList.Add(pathData);
    }


    //LSR
    void Get_LSR_Length()
    {
        //Find both tangent positions
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;

        DubinsMath.RSLorLSR(sLeft, gRight, true, out startTangent, out goalTangent);

        //Calculate lengths
        float length1 = DubinsMath.GetArcLength(sLeft, startPos, startTangent, true);

        float length2 = (startTangent - goalTangent).magnitude;

        float length3 = DubinsMath.GetArcLength(gRight, goalTangent, goalPos, false);

        //Save the data
        DubinsPath pathData = new DubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LSR);

        //We also need this data to simplify when generating the final path
        pathData.segment2Turning = false;

        //LSR
        pathData.SetIfTurningRight(false, false, true);

        //Add the path to the collection of all paths
        pathList.Add(pathData);
    }


    //RLR
    void Get_RLR_Length()
    {
        //Find both tangent positions and the position of the 3rd circle
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;
        //Center of the 3rd circle
        Vector3 middleCircle = Vector3.zero;

        DubinsMath.GetRLRorLRLTangents(
            sRight,
            gRight,
            false,
            out startTangent,
            out goalTangent,
            out middleCircle);

        //Calculate lengths
        float length1 = DubinsMath.GetArcLength(sRight, startPos, startTangent, false);

        float length2 = DubinsMath.GetArcLength(middleCircle, startTangent, goalTangent, true);

        float length3 = DubinsMath.GetArcLength(gRight, goalTangent, goalPos, false);

        //Save the data
        DubinsPath pathData = new DubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RLR);

        //We also need this data to simplify when generating the final path
        pathData.segment2Turning = true;

        //RLR
        pathData.SetIfTurningRight(true, false, true);

        //Add the path to the collection of all paths
        pathList.Add(pathData);
    }


    //LRL
    void Get_LRL_Length()
    {
        //Find both tangent positions and the position of the 3rd circle
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;
        //Center of the 3rd circle
        Vector3 middleCircle = Vector3.zero;

        DubinsMath.GetRLRorLRLTangents(
            sLeft,
            gLeft,
            true,
            out startTangent,
            out goalTangent,
            out middleCircle);

        //Calculate the total length of this path
        float length1 = DubinsMath.GetArcLength(sLeft, startPos, startTangent, true);

        float length2 = DubinsMath.GetArcLength(middleCircle, startTangent, goalTangent, false);

        float length3 = DubinsMath.GetArcLength(gLeft, goalTangent, goalPos, true);

        //Save the data
        DubinsPath pathData = new DubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LRL);

        //We also need this data to simplify when generating the final path
        pathData.segment2Turning = true;

        //LRL
        pathData.SetIfTurningRight(false, true, false);

        //Add the path to the collection of all paths
        pathList.Add(pathData);
    }

    void GeneratePathCoordinates()
    {
        for (int i = 0; i < pathList.Count; i++)
        {
            GetTotalPath(pathList[i]);
        }
    }

    //Find coordinates for the path from the tangents and lengths of each segment

    void GetTotalPath(DubinsPath pathData)
    {
        // Where is the car going?
        List<Vector3> finalPath = new List<Vector3>();

        // Where does the car start?
        Vector3 currentPos = startPos;
        // Where is it heading?
        float theta = startHeading;

        // Add the first location of the car
        finalPath.Add(currentPos);

        int segments = 0;

        segments = Mathf.FloorToInt(pathData.length1 / DubinsMath.driveDistance);

        GenerateDrivingDirections.AddCoordinatesToPath(
            ref currentPos,
            ref theta,
            finalPath,
            segments,
            true,
            pathData.segment1TurningRight);

        segments = Mathf.FloorToInt(pathData.length2 / DubinsMath.driveDistance);

        GenerateDrivingDirections.AddCoordinatesToPath(
            ref currentPos,
            ref theta,
            finalPath,
            segments,
            pathData.segment2Turning,
            pathData.segment2TurningRight);

        segments = Mathf.FloorToInt(pathData.length3 / DubinsMath.driveDistance);

        GenerateDrivingDirections.AddCoordinatesToPath(
            ref currentPos,
            ref theta,
            finalPath,
            segments,
            true,
            pathData.segment3TurningRight);

        //Add the final goal
        finalPath.Add(new Vector3(goalPos.x, currentPos.y, goalPos.z));

        //save this path
        pathData.pathCoords = finalPath;
    }


    public static void AddCoordinatesToPath(
        ref Vector3 currentPos,
        ref float theta,
        List<Vector3> finalPath,
        int segments,
        bool isTurning,
        bool isTurningRight)
    {
        for (int i = 0; i < segments; i++)
        {
            //Update the position of the car
            currentPos.x += DubinsMath.driveDistance * Mathf.Sin(theta);
            currentPos.z += DubinsMath.driveDistance * Mathf.Cos(theta);

            //Don't update the heading if we are driving straight
            if (isTurning)
            {
                //Which way are we turning?
                float turnParameter = 1f; // TODO check this. I think it just makes turning positive or negative, but check in unity that this is correct.

                if (!isTurningRight)
                {
                    turnParameter = -1f;
                }

                //Update the heading
                theta += (DubinsMath.driveDistance / DubinsMath.turningRadius) * turnParameter;
            }

            //Add the new coordinate to the path
            finalPath.Add(currentPos);
        }
    }
    
}
