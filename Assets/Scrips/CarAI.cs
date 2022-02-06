using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using static Node;
using static DubinsPath;
using static GenerateDrivingDirections;


namespace UnityStandardAssets.Vehicles.Car
{

    [RequireComponent(typeof(CarController))]

    

    public class CarAI : MonoBehaviour
    {

        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public Graph graph;
        GenerateDrivingDirections dubinsPathGenerator;
        private Vector3 targetPosition;
        float steeringAmount;
        float accelerationAmount;
        float handbrake;
        float footbrake;
        int nodeNumber;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            DubinsMath.rb = GetComponent<Rigidbody>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Plan your path here
            // Replace the code below that makes a random path
            // ...

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;
            nodeNumber = 0; // We start at the beginning of the path

            List<Vector3> my_path = new List<Vector3>();

            my_path.Add(start_pos);

            for (int i = 0; i < 3; i++)
            {
                Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
                my_path.Add(waypoint);
            }
            my_path.Add(goal_pos);
            Debug.Log(terrain_manager.myInfo.traversability.GetLength(0));
            Debug.Log(terrain_manager.myInfo.traversability.GetLength(1));
            Debug.Log(terrain_manager.myInfo.traversability.Length);
            Debug.Log(terrain_manager.myInfo.traversability.Rank);
            string traversability_string = "";
            /*
            Vector3 carSize = new Vector3(2f, 0.81f, 2f)*2;
            Vector3 granularity = new Vector3((terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / carSize.x, 1, (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / carSize.z);

            Vector3 cube_size = new Vector3((terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low)/ terrain_manager.myInfo.x_N, 0, (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N);
            granularity = cube_size / 2;
            Debug.Log("Granularity x: "+(int)(granularity.x));
            Debug.Log("Granularity z: "+(int)(granularity.z));

            graph = Graph.CreateGraph(terrain_manager.myInfo, (int)(granularity.x), (int)(granularity.z));
            */
            graph = Graph.CreateGraph(terrain_manager.myInfo, terrain_manager.myInfo.traversability.GetLength(0)*2, terrain_manager.myInfo.traversability.GetLength(1) * 2);

            for (int i = 0; i < terrain_manager.myInfo.traversability.GetLength(0); i++)
            {
                for (int j = 0; j < terrain_manager.myInfo.traversability.GetLength(1); j++)
                {
                    traversability_string += (terrain_manager.myInfo.traversability[i, j] + " ");
                    if (terrain_manager.myInfo.traversability[i, j] == 0 && terrain_manager.myInfo.traversability[i, j+1]==0)
                    {
                        if (graph.AreConnected(new Node(i, j, 0,0), new Node(i, j+1, 0,0)))
                        {
                            //Debug.Log("Connected");
                        }
                        else
                        {
                            //Debug.Log("ERROR!!!!!");
                        }
                      
                    }
                }
                traversability_string += "\n";
            }
            //Debug.Log(traversability_string);
            //Debug.Log("HERE!");
            graph.printTraversability();



            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }

            PathFinder.findPath(graph, start_pos, goal_pos); // path is accessible through graph.path

            //dubinsPathGenerator = new GenerateDrivingDirections(m_Car);

        }

        void OnDrawGizmos() // draws grid on map and shows car
        {
            //Debug.Log("HERE!!!");
            if (graph != null)
            {
                //Debug.Log("HERE TOO!!!");
                foreach (Node n in graph.nodes) // graph.path 
                {
                    Gizmos.color = (n.walkable) ? Color.blue : Color.red;
                    if(graph.path != null && graph.path.Contains(n))
                        Gizmos.color = Color.green;
                    Gizmos.DrawCube(n.worldPosition, new Vector3(graph.x_unit*0.8f, 0.5f, graph.z_unit*0.8f));
                }

                Node currentNode = graph.getNodeFromPoint(transform.position);
                Gizmos.color = Color.cyan; // position of car
                Gizmos.DrawCube(currentNode.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
            }
        }

        private void MoveDubins()
        {
            //We need some information to use Dubins paths, start_pos, next position in the path, and headings.
            Vector3 start;
            Vector3 end;
            start = transform.position;
            float startHeading = transform.eulerAngles.y * Mathf.Deg2Rad;
            float endHeading;
            float turnLeft = m_Car.m_MaximumSteerAngle * -1f;
            float turnRight = m_Car.m_MaximumSteerAngle;
            
            foreach (Node n in graph.path)
            {
                if (n != null)
                {
                    //update the new ending position
                    
                    end.x = n.x_pos;
                    end.y = start.y;
                    end.z = n.z_pos;
                    endHeading = n.heading;

                    List<DubinsPath> pathList = dubinsPathGenerator.makeManyDubinsPaths(
                        start,
                        startHeading,
                        end,
                        endHeading);

                    if (pathList.Count > 0)
                    {
                        foreach (DubinsPath path in pathList)
                        {
                            Debug.Log("Current path type: " + path.pathType);
                            switch(path.pathType)
                            { 
                                case GenerateDrivingDirections.PathType.LRL:
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.RLR:
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.LSR:
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    m_Car.Move(0f, 1f, 1f, 0f);
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.LSL:
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    m_Car.Move(0f, 1f, 1f, 0f);
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.RSL:
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    m_Car.Move(0f, 1f, 1f, 0f);
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.RSR:
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    m_Car.Move(0f, 1f, 1f, 0f);
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    break;
                             }

                        }

                    }
                    start = end;
                    
                }
            }
        }


        public void SetNextTarget(Vector3 targetPosition)
        {
            this.targetPosition = targetPosition;
        }

        public void setHandbrake()
        {
            this.handbrake = 0f;
        }

        

        public void SetAccelerationSteering()
        {
            Vector3 directionToMove = (this.targetPosition - transform.position).normalized;
            float dot = Vector3.Dot(transform.forward, directionToMove);
            float steeringAngle = Vector3.SignedAngle(transform.forward, directionToMove, Vector3.up);

            if (dot > 0)
            {
                this.accelerationAmount=1f;
                this.footbrake = 0f;
            }
            else
            {
                this.accelerationAmount = 0f; // this doesn't work because the acceleration is clamped to 0,1.
                this.footbrake = -1f;
            }

            steeringAngle = Mathf.Clamp(steeringAngle, -25, 25);
            this.steeringAmount = steeringAngle / m_Car.m_MaximumSteerAngle;
        }
        private void FixedUpdate()
        {
            // this is how you access information about the terrain from the map
            //int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            //int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            //float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            //float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));
            setHandbrake();
            Node n = graph.path[nodeNumber];

            float targetDistanceMargin = 5f;
            Vector3 nextPosition = new Vector3(n.x_pos, transform.position.y, n.z_pos);
            Debug.Log("Next position is " + nextPosition);
            SetNextTarget(nextPosition);
            float distanceToTarget = Vector3.Distance(transform.position, targetPosition);
            if (distanceToTarget > targetDistanceMargin)
            {
                SetAccelerationSteering();
                Debug.Log("Acceleration is set to " + accelerationAmount);
                Debug.Log("Steering is set to " + steeringAmount);
                m_Car.Move(steeringAmount, accelerationAmount, footbrake, handbrake);
            }
            else //we reached the waypoint or end point
            {
                if (targetPosition == terrain_manager.myInfo.goal_pos) // we made it to the end, stop the car
                {
                    m_Car.Move(0f, 0f, 0f, 0f);
                }
                else // we arrived at a waypoint node, move to the next one
                {
                    nodeNumber += 1;
                }
            }

                   
/*
            // this is how you access information about the terrain from a simulated laser range finder
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                //Debug.Log("Hit " + hit.collider.gameObject.name + " at " + hit.distance);
                //Debug.Log(terrain_manager.myInfo.traversability[2, 2]);
            }
*/

            // this is how you control the car
            // public void Move(float steering, float accel, float footbrake, float handbrake)
            // we can access the dimension of the terrain and of each block

            //m_Car.Move(1f, 1f, 1f, 0f);
            //Debug.Log("Max steering angle: " + m_Car.m_MaximumSteerAngle);


        }
    }
}
