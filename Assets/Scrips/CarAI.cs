using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using static Node;


namespace UnityStandardAssets.Vehicles.Car
{

    [RequireComponent(typeof(CarController))]

    

    public class CarAI : MonoBehaviour
    {

        private CarController m_Car; // the car controller we want to use
        public List<NewNode> path;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public Graph graph;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

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
            /*Debug.Log(terrain_manager.myInfo.traversability.GetLength(0));
            Debug.Log(terrain_manager.myInfo.traversability.GetLength(1));
            Debug.Log(terrain_manager.myInfo.traversability.Length);
            Debug.Log(terrain_manager.myInfo.traversability.Rank);
            */
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
            float initial_orientation = 90f;
            //PathFinder.findPath(graph, new Vector4(start_pos.x, start_pos.y, start_pos.z, initial_orientation), new Vector4(goal_pos.x, goal_pos.y, goal_pos.z, initial_orientation),PathFinder.getSteeringSteps(25f, 1), 20, 10) ;

            path = PathFinder.FindPath2(graph, new Vector4(start_pos.x, start_pos.y, start_pos.z, initial_orientation), new Vector4(goal_pos.x, goal_pos.y, goal_pos.z, initial_orientation), PathFinder.getSteeringSteps(25f, 1), 20, 10);


            if (graph.path != null)
            {
                foreach (Node n in graph.path)
                {
                    Debug.Log("Current node pos: (" + n.x_pos + "," + n.z_pos + ")");
                }
            }

        }

        void OnDrawGizmos()
        {
            if (graph != null)
            {
                foreach (Node n in graph.nodes)
                {
                    float n_x = n.x_pos;
                    float n_z = n.z_pos;
                    Gizmos.color = (n.walkable) ? Color.blue : Color.red;
                    /*if(path != null)
                    {
                        foreach(NewNode node in path)
                        {
                            float x = node.continous_position.x;
                            float z = node.continous_position.z;
                            if (x<= n_x + Graph.x_unit && x<= n_x - Graph.x_unit && z <= n_z + Graph.z_unit && z <= n_z - Graph.z_unit)
                            {
                                Debug.Log("Pos of GREEN node: " + node.continous_position);

                                Gizmos.color = Color.green;
                                break;
                            }
                            else
                            {
                                Debug.Log("Pos of node: " + node.continous_position);

                            }
                        }
                    }*/
                        
                    Gizmos.DrawCube(n.worldPosition, new Vector3(Graph.x_unit*0.8f, 0.5f, Graph.z_unit*0.8f));
                }

                Node currentNode = graph.getNodeFromPoint(transform.position);
                Gizmos.color = Color.cyan;
                Gizmos.DrawCube(currentNode.worldPosition, new Vector3(Graph.x_unit * 0.8f, 0.5f, Graph.z_unit * 0.8f));
            }
            /*
            for(int i = 1; i< PathFinder.visited_points.Count; i++)
            {
                Gizmos.DrawLine(PathFinder.visited_points[i - 1].continousPosition, PathFinder.visited_points[i].continousPosition);
            }
            */

            /*for (int i = 1; i < PathFinder.newVisited_points.Count; i++)
            {
                Gizmos.color = Color.yellow;

                Vector3 start = PathFinder.newVisited_points[i - 1].continous_position;
                Vector3 end = PathFinder.newVisited_points[i].continous_position;
                Gizmos.DrawLine(start, end);
                Debug.Log("Starting from: " + start + "walkable: " + Graph.isWalkable(start) +" to end: " + end + " walkable: " + Graph.isWalkable(end));

            }
            */
            

            for (int i = 1; i < path.Count; i++)
            {
                Gizmos.color = Color.green;

                Vector3 start = path[i - 1].continous_position;
                Vector3 end = path[i].continous_position;
                Gizmos.DrawLine(start, end);
                Debug.Log("Starting from: " + start + "walkable: " + Graph.isWalkable(start) + " to end: " + end + " walkable: " + Graph.isWalkable(end));

            }

            Gizmos.DrawLine(path[path.Count-1].continous_position, graph.goal_node.worldPosition);



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


            // this is how you control the car
            // public void Move(float steering, float accel, float footbrake, float handbrake)
            // we can access the dimension of the terrain and of each block

            m_Car.Move(1f, 1f, 1f, 0f);
            
            //Debug.Log("Max steering angle: " + m_Car.m_MaximumSteerAngle);


        }

    }

}
