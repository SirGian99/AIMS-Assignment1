﻿using System.Collections;
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
        List<Node> up_and_smooth;
        List<Node> bez_path;
        Vector3 target_velocity;
        Vector3 target_position;
        Vector3 old_target_pos;
        public Rigidbody rigidbody;
        public float k_p = 1.2f;
        public float k_d = 0.26f;
        int node_index = 1;
        int max_stuck_safe_exit = 10000;
        int stuck_safe_exit = 10000;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            rigidbody = GetComponent<Rigidbody>();
            DubinsMath.rb = rigidbody;

            // Plan your path here
            // Replace the code below that makes a random path
            // ...

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

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
            int x_scale = terrain_manager.myInfo.x_N;
            int z_scale = terrain_manager.myInfo.z_N;
            float ratio = ((terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / x_scale) / ((terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low)/z_scale);
            if (ratio < 1)
            {
                
                ratio = (1 / ratio);
                z_scale *= (int)ratio;
                Debug.Log("Rescaling z of " + ratio + " going from " + terrain_manager.myInfo.z_N + " to " + z_scale);
            }
            else
            {
                x_scale *= (int)ratio;
                Debug.Log("Rescaling x of " + ratio + " going from " + terrain_manager.myInfo.x_N + " to " + x_scale);
            }

            Debug.Log("Final block size: " + (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / z_scale);


            graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);

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
            bez_path = PathFinder.bezierPath(graph.path, 2);

            up_and_smooth = PathFinder.pathSmoothing(PathFinder.pathUpsampling(graph.path, 2),0.6f, 0.2f, 1E-09f);
            graph.path = PathFinder.pathSmoothing(graph.path);

            //dubinsPathGenerator = new GenerateDrivingDirections(m_Car);

            
            /*////TODO Path smoothr test
            List<Node> path_to_smooth = new List<Node>();

            path_to_smooth.Add(new Node(1, 1, 0, 0));
            path_to_smooth.Add(new Node(1, 1, 0, 1));
            path_to_smooth.Add(new Node(1, 1, 0, 2));
            path_to_smooth.Add(new Node(1, 1, 1, 2));
            path_to_smooth.Add(new Node(1, 1, 2, 2));
            path_to_smooth.Add(new Node(1, 1, 3, 2));
            path_to_smooth.Add(new Node(1, 1, 4, 2));
            path_to_smooth.Add(new Node(1, 1, 4, 3));
            path_to_smooth.Add(new Node(1, 1, 4, 4));

            List<Node> smoothed = PathFinder.pathSmoothing(path_to_smooth);

            foreach(Node n in smoothed)
            {
                Debug.Log("SMOOTH [" + n.x_pos + "," + n.z_pos + "]" );
            }
            */
        }

        void OnDrawGizmos() // draws grid on map and shows car
        {
            if (graph != null)
            {
                ////TODO UNCOMMENT THIS LOOP TO SEE THE BLOCKS OF THE GRID
                /*foreach (Node n in graph.nodes) // graph.path 
                {
                    Gizmos.color = (n.walkable) ? Color.blue : Color.red;
                    if(graph.path != null && graph.path.Contains(n))
                        Gizmos.color = Color.white;
                    Gizmos.DrawCube(n.worldPosition, new Vector3(graph.x_unit*0.8f, 0.5f, graph.z_unit*0.8f));
                }
                */

                Node currentNode = graph.getNodeFromPoint(transform.position);
                //Debug.Log("CAR INITIAL NODE: [" + currentNode.i + "," + currentNode.j + "]");
                Gizmos.color = Color.cyan; // position of car
                Gizmos.DrawCube(currentNode.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
            }

            if (graph.path != null)
            {
                for(int i=0; i<graph.path.Count-1; i++)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(graph.path[i].worldPosition, graph.path[i + 1].worldPosition);
                }
                
                List<Node> augmented_path = up_and_smooth;//PathFinder.pathUpsampling(graph.path, 8);

                for (int i = 0; up_and_smooth!=null && i < augmented_path.Count; i++)
                {
                    Gizmos.color = Color.black;
                    //Gizmos.DrawSphere(augmented_path[i].worldPosition, 0.5f);
                    Gizmos.DrawLine(augmented_path[i].worldPosition, augmented_path[i + 1].worldPosition);
                    //Gizmos.DrawLine(augmented_path[i].worldPosition, augmented_path[i + 1].worldPosition);
                }

                /*for (int i = 0; i < graph.path.Count; i++)
                {
                    Gizmos.color = Color.red;
                    //Gizmos.DrawSphere(graph.path[i].worldPosition, 0.5f);
                    Gizmos.DrawLine(graph.path[i].worldPosition, graph.path[i + 1].worldPosition);
                    //Gizmos.DrawLine(augmented_path[i].worldPosition, augmented_path[i + 1].worldPosition);
                }
                */

                if(bez_path!=null)
                for (int i = 0; i < bez_path.Count-1; i++)
                {
                    Gizmos.color = Color.black;
                    //Gizmos.DrawSphere(augmented_path[i].worldPosition, 0.5f);
                    Gizmos.DrawLine(bez_path[i].worldPosition, bez_path[i + 1].worldPosition);
                    //Gizmos.DrawLine(augmented_path[i].worldPosition, augmented_path[i + 1].worldPosition);
                    Debug.Log( "i: " + i + " Current point: [" + bez_path[i].x_pos + "," + bez_path[i].z_pos + "]");
                }                                      
            }


        }

        private void FixedUpdate()
        {
            
            // Execute your path here
            // ...


            // this is how you access information about the terrain from the map
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));

            //We need some information to use Dubins paths, start_pos, next position in the path, and headings.
            Vector3 start;
            Vector3 end;
            start = transform.position;
            float startHeading = transform.eulerAngles.y * Mathf.Deg2Rad;
            float endHeading;
            float turnLeft = m_Car.m_MaximumSteerAngle * -1f;
            float turnRight = m_Car.m_MaximumSteerAngle;

            if (false && graph.path != null) {
                foreach (Node n in graph.path)
                {
                    if (n != null)
                    {
                        //update the new ending position

                        end.x = n.x_pos;
                        end.y = start.y;
                        end.z = n.z_pos;
                        endHeading = 0; // n.heading;

                        List<DubinsPath> pathList = dubinsPathGenerator.makeManyDubinsPaths(
                            start,
                            startHeading,
                            end,
                            endHeading);

                        if (pathList != null && pathList.Count > 0)
                        {
                            foreach (DubinsPath path in pathList)
                            {
                                switch (path.pathType)
                                {
                                    case PathType.LRL:
                                        m_Car.Move(turnLeft, 1f, 1f, 0f);
                                        m_Car.Move(turnRight, 1f, 1f, 0f);
                                        m_Car.Move(turnLeft, 1f, 1f, 0f);
                                        break;
                                    case PathType.RLR:
                                        m_Car.Move(turnRight, 1f, 1f, 0f);
                                        m_Car.Move(turnLeft, 1f, 1f, 0f);
                                        m_Car.Move(turnRight, 1f, 1f, 0f);
                                        break;
                                    case PathType.LSR:
                                        m_Car.Move(turnLeft, 1f, 1f, 0f);
                                        m_Car.Move(0f, 1f, 1f, 0f);
                                        m_Car.Move(turnRight, 1f, 1f, 0f);
                                        break;
                                    case PathType.LSL:
                                        m_Car.Move(turnLeft, 1f, 1f, 0f);
                                        m_Car.Move(0f, 1f, 1f, 0f);
                                        m_Car.Move(turnLeft, 1f, 1f, 0f);
                                        break;
                                    case PathType.RSL:
                                        m_Car.Move(turnRight, 1f, 1f, 0f);
                                        m_Car.Move(0f, 1f, 1f, 0f);
                                        m_Car.Move(turnLeft, 1f, 1f, 0f);
                                        break;
                                    case PathType.RSR:
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

            if (up_and_smooth != null && i < up_and_smooth.Count)
            {
                if(inRange(old_target_pos, new Vector3(rigidbody.position.x, 0, rigidbody.position.z), 1))
                    stuck_safe_exit--;
                else
                    stuck_safe_exit = max_stuck_safe_exit;

                old_target_pos = new Vector3(rigidbody.position.x, 0, rigidbody.position.z);
              
                

                Vector3 target_position = up_and_smooth[node_index].worldPosition;

                float slow_down_rate = 1;

                int backup = get_closest_node(old_target_pos, up_and_smooth);
                int tolerance = 10;
                if(node_index-tolerance<backup && backup < node_index + tolerance && stuck_safe_exit!=0)
                {
          
                    target_position = up_and_smooth[backup].worldPosition;
                }

                if(stuck_safe_exit == 0)
                {
                    Debug.Log("EIEIEI");
                    for(int k = 0; k<100; k++)
                    {
                        
                        m_Car.Move(0.1f, 1f, 0, 0);
                    }

                    return;

                    target_position = up_and_smooth[node_index+2].worldPosition;
                }
                target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
                Debug.Log(node_index + ")Target: " + target_position + " Old:" + old_target_pos + " Vel: " + target_velocity + " Car_pos: " + rigidbody.position);
                //old_target_pos = target_position;

                // a PD-controller to get desired velocity
                Vector3 position_error = target_position - transform.position;
                Vector3 velocity_error = target_velocity/2 - rigidbody.velocity;
                Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

                float steering = Vector3.Dot(desired_acceleration, transform.right);
                float acceleration = Vector3.Dot(desired_acceleration, transform.forward)/2;

                Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
                Debug.DrawLine(transform.position, transform.position + rigidbody.velocity, Color.blue);
                Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

                // this is how you control the car
                Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
                if (Math.Abs(steering) > m_Car.m_MaximumSteerAngle / 2 && stuck_safe_exit!= 0)
                {
                    print("HUGE: " + steering + " STuck: " + stuck_safe_exit);
                    slow_down_rate = 1.0001f - steering / m_Car.m_MaximumSteerAngle;
                }

                RaycastHit hit;
                float maxRange = 5f;
                if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
                {
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    Debug.Log("Hit " + hit.collider.gameObject.name + " at " + hit.distance);

                    if (hit.distance <= 10f)
                    {
                        acceleration /= 100;
                        if (hit.distance < 5f)
                        {
                            m_Car.Move(0f, -1f, 0f, 0f);
                        }
                    }
                    //Debug.Log(terrain_manager.myInfo.traversability[2, 2]);
                }

                if (Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.right), out hit, maxRange))
                {
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.right) * hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    m_Car.Move(0f, 0f, 1f, 0f);
                    Debug.Log("Hit RIGHT " + hit.collider.gameObject.name + " at " + hit.distance);

                    if (hit.distance <= 10f)
                    {
                        steering = -1;
                    }
                    //Debug.Log(terrain_manager.myInfo.traversability[2, 2]);
                }

                if (Physics.Raycast(transform.position - transform.right, transform.TransformDirection(Vector3.left), out hit, maxRange))
                {
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.left) * hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    m_Car.Move(0f, 0f, 1f, 0f);
                    Debug.Log("Hit LEFT" + hit.collider.gameObject.name + " at " + hit.distance);

                    if (hit.distance <= 10f)
                    {
                        if (steering == -1)
                            steering = 0;
                        else steering = 1;
                    }
                    //Debug.Log(terrain_manager.myInfo.traversability[2, 2]);
                }

                m_Car.Move(steering, acceleration * slow_down_rate, acceleration * slow_down_rate, 0f);
                if (inRange(rigidbody.position, target_position, 2.5f))
                {
                    node_index++;
                    Debug.Log("Node " + (node_index - 1) + " reached");
                }

                if (stuck_safe_exit == 0)
                    stuck_safe_exit = max_stuck_safe_exit;
            }
            else { Debug.Log("OH NO"); }

            
                        // this is how you access information about the terrain from a simulated laser range finder
            
            

            // this is how you control the car
            // public void Move(float steering, float accel, float footbrake, float handbrake)
            // we can access the dimension of the terrain and of each block

            //m_Car.Move(1f, 1f, 1f, 0f);
            //Debug.Log("Max steering angle: " + m_Car.m_MaximumSteerAngle);


        }

        public bool inRange(Vector3 current_pos, Vector3 target_pos, float range)
        {
            return (Math.Abs(current_pos.x - target_pos.x) + Math.Abs(current_pos.z - target_pos.z))<=range;
        }

        public int get_closest_node(Vector3 position, List<Node> path)
        {
            int closest = 0;
            float distance = 1000000000;
            int safe_exit = 5;
            for (int i = 0; i<path.Count && safe_exit>0; i++)
            {
                float new_distance = Vector3.Distance(position, path[i].worldPosition);
                if (new_distance <= distance) {
                    distance = new_distance;
                    closest = i;
                    }
                else
                {
                    safe_exit--;
                }
            }
            return closest;
        }
    }

}
