using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace UnityStandardAssets.Vehicles.Car
{

    [RequireComponent(typeof(CarController))]

    

    public class CarAI : MonoBehaviour
    {

         public class Node {
        public int i;
        public int j;
        public float x_pos;
        public float z_pos;
        public Node(int i, int j, float x_pos, float z_pos)
        {
            this.i = i;
            this.j = j;
            this.x_pos = x_pos;
            this.z_pos = z_pos;
        }

        // override object.Equals
        public override bool Equals(object obj)
        {
            //
            // See the full list of guidelines at
            //   http://go.microsoft.com/fwlink/?LinkID=85237
            // and also the guidance for operator== at
            //   http://go.microsoft.com/fwlink/?LinkId=85238
            //
            
            if (obj == null || GetType() != obj.GetType())
            {
                return false;
            }
            

            return i == ((Node)obj).i && j == ((Node)obj).j; //////TODO add check also on positions
        }
        
        // override object.GetHashCode
        public override int GetHashCode()
        {

            return base.GetHashCode();
        }
    }

   

    public class GraphEdge{
            
            public Node start_node;
            public Node end_node;
            public float cost;
    
            public GraphEdge(Node start_node, Node end_node, float cost){
                this.start_node = start_node;
                this.end_node = end_node;
                this.cost = cost;
            }
            public GraphEdge(Node start_node, Node end_node){
                this.start_node = start_node;
                this.end_node = end_node;
                this.cost = 0;
            }
            // override object.Equals
            public override bool Equals(object obj)
            {
                //
                // See the full list of guidelines at
                //   http://go.microsoft.com/fwlink/?LinkID=85237
                // and also the guidance for operator== at
                //   http://go.microsoft.com/fwlink/?LinkId=85238
                //
                
                if (obj == null || GetType() != obj.GetType())
                {
                    return false;
                }
                
                return start_node.Equals(((GraphEdge)obj).start_node) && end_node.Equals(((GraphEdge)obj).end_node) || start_node.Equals(((GraphEdge)obj).end_node) && end_node.Equals(((GraphEdge)obj).start_node);
            }
            
            // override object.GetHashCode
            public override int GetHashCode()
            {
                return base.GetHashCode();
            }
    }

    public class Graph{
        public int i_size;
        public int j_size;
        public float x_low;
        public float x_high;
        public float z_low;
        public float z_high;
        public float x_unit;
        public float z_unit;
        public Node start_node;
        public Node goal_node;

        public HashSet<GraphEdge> edges;

        public Graph(int i_size, int j_size, float x_low, float x_high, float z_low, float z_high)
        {
            this.i_size = i_size;
            this.j_size = j_size;
            this.x_low = x_low;
            this.x_high = x_high;
            this.z_low = z_low;
            this.z_high = z_high;
            this.x_unit = (x_high - x_low) / i_size;
            this.z_unit = (z_high - z_low) / j_size;
            this.edges = new HashSet<GraphEdge>();
        }

        public void AddEdge(GraphEdge edge)
            {
                edges.Add(edge);
            }

        public bool AreConnected(Node node1, Node node2)
        {
            foreach (GraphEdge edge in edges)
            {
                if (edge.start_node.Equals(node1) && edge.end_node.Equals(node2))
                {
                    return true;
                }
                if (edge.start_node.Equals(node2) && edge.end_node.Equals(node1))
                {
                    return true;
                }
            }
            return false;
        }
        
    }

        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public static Graph CreateGraph(TerrainInfo terrain_manager , int x_N, int z_N){
            if (terrain_manager == null){
                Debug.Log("terrain_manager is null");
                return null;
            }

            if (x_N <= 0 || z_N <= 0){
                Debug.Log("x_N or z_N is less than or equal to 0");
                return null;
            }

            Debug.Log("myFunction");
            Graph graph = new Graph(x_N, z_N, terrain_manager.x_low, terrain_manager.x_high, terrain_manager.z_low, terrain_manager.z_high);
            float x_len = terrain_manager.x_high - terrain_manager.x_low;
            float z_len = terrain_manager.z_high - terrain_manager.z_low;
            float x_unit = x_len/x_N;
            float z_unit = z_len/z_N;
            graph.start_node = new Node((int)(terrain_manager.start_pos[0]/x_unit), (int)(terrain_manager.start_pos[2]/z_unit), terrain_manager.start_pos[0], terrain_manager.start_pos[2]);
            graph.goal_node = new Node((int)(terrain_manager.goal_pos[0]/x_unit), (int)(terrain_manager.goal_pos[2]/z_unit), terrain_manager.goal_pos[0], terrain_manager.goal_pos[2]);

            for (int i = 0; i < x_N-1; i++){
                for (int j = 0; j < z_N-1; j++){
                    float x_center = terrain_manager.x_low + x_unit * (i + 0.5f);
                    float z_center = terrain_manager.z_low + z_unit * (j + 0.5f);
                    Node node = new Node(i, j, x_center, z_center);

                    int i_index = terrain_manager.get_i_index(x_center);
                    int j_index = terrain_manager.get_j_index(z_center);
                    if (terrain_manager.traversability[i_index, j_index] == 0){
                        int next_i_index = terrain_manager.get_i_index(x_center + x_unit); //right
                        if (terrain_manager.traversability[next_i_index, j_index] == 0){
                            Node right_node = new Node(i+1, j, x_center + x_unit, z_center);
                            GraphEdge edge = new GraphEdge(node, right_node);
                            GraphEdge edge_reverse = new GraphEdge(right_node, node);
                            graph.AddEdge(edge);
                            graph.AddEdge(edge_reverse);
                            Debug.Log("RIGHT:" + "Adding edge from node (" + node.i + "," + node.j + ") to node (" + right_node.i + "," + right_node.j + ")");
                        } else {
                            Debug.Log("RIGHT:" + "Not adding edge from node (" + i + "," + j + ") to node (" + (i+1) + "," + j + ")");
                        }

                        int next_j_index = terrain_manager.get_j_index(z_center + z_unit); //up
                        if (terrain_manager.traversability[i_index, next_j_index] == 0){
                            Node up_node = new Node(i, j+1, x_center, z_center + z_unit);
                            GraphEdge edge = new GraphEdge(node, up_node);
                            GraphEdge edge_reverse = new GraphEdge(up_node, node);
                            graph.AddEdge(edge);
                            graph.AddEdge(edge_reverse);
                            Debug.Log("UP: Adding edge from node (" + node.i + "," + node.j + ") to node (" + up_node.i + "," + up_node.j + ")");

                        }
                        else {
                            Debug.Log("RIGHT:" + "Not adding edge from node (" + i + "," + j + ") to node (" + (i) + "," +(j+1) + ")");
                        }


                        /*int next_i_index = terrain_manager.get_i_index(x_center - x_unit); //left
                        if (terrain_manager.traversability[next_i_index, j_index] == 0){
                            Node left_node = new Node(i-1, j, x_center - x_unit, z_center);
                            GraphEdge edge = new GraphEdge(node, left_node);
                        }
                        int next_j_index = terrain_manager.get_j_index(z_center + z_unit); //down
                        if (terrain_manager.traversability[i_index, next_j_index] == 0){
                            Node down_node = new Node(i, j-1, x_center, z_center - z_unit);
                            GraphEdge edge = new GraphEdge(node, down_node);
                        }
                        */

                    }

                }
            }
            return graph;
        }

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
            Debug.Log(terrain_manager.myInfo.traversability.GetLength(0));
            Debug.Log(terrain_manager.myInfo.traversability.GetLength(1));
            Debug.Log(terrain_manager.myInfo.traversability.Length);
            Debug.Log(terrain_manager.myInfo.traversability.Rank);
            string traversability_string = "";

            Graph graph = CreateGraph(terrain_manager.myInfo, terrain_manager.myInfo.traversability.GetLength(0), terrain_manager.myInfo.traversability.GetLength(1));

            for (int i = 0; i < terrain_manager.myInfo.traversability.GetLength(0); i++)
            {
                for (int j = 0; j < terrain_manager.myInfo.traversability.GetLength(1); j++)
                {
                    traversability_string += (terrain_manager.myInfo.traversability[i, j] + " ");
                    if (terrain_manager.myInfo.traversability[i, j] == 0 && terrain_manager.myInfo.traversability[i, j+1]==0)
                    {
                        if (graph.AreConnected(new Node(i, j, 0,0), new Node(i, j+1, 0,0)))
                        {
                            Debug.Log("Connected");
                        }
                        else
                        {
                            Debug.Log("ERROR!!!!!");
                        }
                      
                    }
                }
                traversability_string += "\n";
            }
            Debug.Log(traversability_string);



            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
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
            Debug.Log("Max steering angle: " + m_Car.m_MaximumSteerAngle);


        }
    }
}
