using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class PathFinder : MonoBehaviour
{

    public static List<ContinousNode> visited_points = new List<ContinousNode>();
    public static List<NewNode> newVisited_points = new List<NewNode>();
    public class ContinousNode
    {
        public Node node;
        public Vector4 continousPosition;
        public ContinousNode parent;

        public ContinousNode(Node node, Vector4 position)
        {
            this.node = node;
            continousPosition = position;
        }

        public override bool Equals(object obj)
        {
            if (obj == null || GetType() != obj.GetType())
            {
                return false;
            }
            return node.Equals(((ContinousNode)obj).node); //////TODO add check also on positions
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }

    public static float[] velocity = { -25f,-12.5f, 12.5f, 25f };
    public static float car_len = 1f;

                                
    public static void findPath(Graph graph, Vector4 start_position, Vector4 goal_position, float[] steering_angles, float max_additional_cost, float negative_bias)
    {
        if (Graph.x_unit != 0)
        {
            float diagonal = (float)Math.Sqrt(Graph.x_unit * Graph.x_unit + Graph.z_unit * Graph.z_unit) + 0.01f;
            velocity[0] = -diagonal;
            velocity[1] = -diagonal / 2;
            velocity[2] = diagonal / 2;
            velocity[3] = diagonal;

        }
        ContinousNode start_node = new ContinousNode(graph.getNodeFromPoint(start_position), start_position);

        ContinousNode goal_node = new ContinousNode(graph.getNodeFromPoint(goal_position), goal_position);

        Debug.Log("Start i: " + start_node.node.i + "Start j:" + start_node.node.j);
        Debug.Log("Goal i: " + goal_node.node.i + "Goal j:" + goal_node.node.j);

        List<ContinousNode> open_set = new List<ContinousNode>();
        HashSet<ContinousNode> closed_set = new HashSet<ContinousNode>();
        open_set.Add(start_node);
        bool k = true;

        //Possible input combinations for the car
        int comb_size = steering_angles.Length * velocity.Length;
        Tuple <float, float> [] combinations = new Tuple<float, float>[comb_size];

        Debug.Log("DEBUG Comb len: " + comb_size);
        Debug.Log("DEBUG steering_angles len: " + steering_angles.Length);


        for (int i = 0; i < velocity.Length; i++)
        {
            for(int j = 0; j< steering_angles.Length; j++)
            {
                //Debug.Log("DEBUG i = "+i+" j = "+j+" combinations index: " + (j + i * steering_angles.Length));
                combinations[j + i * steering_angles.Length] = new Tuple<float,float>(velocity[i], steering_angles[j]);
            }
        }

        float[] additional_costs = new float[combinations.Length];
        for (int i = 0; i < additional_costs.Length / 2; i++)
        {
            if (i < additional_costs.Length / 4)
            {
                additional_costs[i] = negative_bias + max_additional_cost - (max_additional_cost * i / (int)(additional_costs.Length / 4));
            }
            else if (i > additional_costs.Length / 4)
            {
                additional_costs[i] = negative_bias + (max_additional_cost * (i - (int)(additional_costs.Length / 4)) / (int)(additional_costs.Length / 4));
            }
            else
            {
                additional_costs[i] = negative_bias;
            }
        }

        for (int i = 0; i < additional_costs.Length/2; i++)
        {
            int index = i + additional_costs.Length / 2;
            if (i < additional_costs.Length / 4)
            {
                additional_costs[index] = max_additional_cost - (max_additional_cost * i / (int)(additional_costs.Length / 4));
            }
            else if (i > additional_costs.Length / 4)
            {
                additional_costs[index] = (max_additional_cost * (i - (int)(additional_costs.Length / 4)) / (int)(additional_costs.Length / 4));
            }
            else
            {
                additional_costs[index] = 0;
            }
        }
        int iteration = -1;


        for (int i = 0; i<additional_costs.Length && iteration == -1; i++)
        {
            print("Additiona cost of combination " + combinations[i] + ": " + additional_costs[i]);
        }

        int iteration_stop = 5;

        while (open_set.Count > 0)
        {
            iteration++;
            ContinousNode current = open_set[0];
            if (!visited_points.Contains(current))
                visited_points.Add(current);
            if (!k)
            {
                Debug.Log("Nodo fuori penalty: " + current.node.wallClosenessCost);
                Debug.Log("Totale: " + current.node.fCost);
            }
            for (int i=1; i<open_set.Count; i++)
            {
                if(open_set[i].node.fCost < current.node.fCost || open_set[i].node.fCost == current.node.fCost && open_set[i].node.hCost < current.node.hCost)
                {
                    current = open_set[i];
                }
            }
            open_set.Remove(current);
            closed_set.Add(current);
            if (current.Equals(goal_node) || iteration==iteration_stop)
            {
                Debug.Log("ARRIVO QUI!");
                List<Node> path = new List<Node>();

                ContinousNode previous_node = current;

                while (!previous_node.Equals(start_node))
                {
                    //Debug.Log("Coordinates (" + previous_node.i + "," + previous_node.j + ")");
                    previous_node.node.realPathPosition = previous_node.continousPosition;
                    path.Add(previous_node.node);
                    previous_node = previous_node.parent;
                }
                path.Reverse();
                graph.path = path;
                //foreach(Node n in graph.path){
                    //Debug.Log("Path, nodo (" + previous_node.i + "," + previous_node.j + ")");
                //}
            }

            Debug.Log("Iteration " + iteration + " current=[" + current.node.i +"," + current.node.j + "] position: (" + current.continousPosition + ")");
            List<ContinousNode> neighbours = getNextNodes(graph, car_len, current.continousPosition, combinations);

            for (int i = 0; i<neighbours.Count; i++)
            {
                ContinousNode neighbour = neighbours[i];
                Debug.Log("Iteration " + iteration + " neighbours[" + i + "]: [" + neighbour.node.i +"," + neighbour.node.j + "] position: (" + neighbour.continousPosition + ")");
                if (iteration == iteration_stop)
                    return;//TODO Remove to trim execution
                     //return;
                if (!neighbour.node.walkable || closed_set.Contains(neighbour))
                    continue;
                float costToNeighb = current.node.gCost + getDistance(graph, current.node, neighbour.node) + additional_costs[i];
                if (costToNeighb < neighbour.node.gCost || !open_set.Contains(neighbour))
                {
                    neighbour.node.gCost = costToNeighb;
                    neighbour.node.hCost = getDistance(graph, neighbour.node, goal_node.node);
                    neighbour.parent = current;
                    if (!open_set.Contains(neighbour))
                        open_set.Add(neighbour);
                    else
                    {
                        open_set.Remove(neighbour);
                        open_set.Add(neighbour);
                    }
                    visited_points.Add(neighbour);
                }
            }

            /* //TODO MODIFICA QUI
            foreach(Node neighbour in current.neighbours)
            {
                if (!neighbour.walkable || closed_set.Contains(neighbour))
                    continue;
                float costToNeighb = current.gCost + getDistance(graph, current, neighbour);
                if (costToNeighb < neighbour.gCost || !open_set.Contains(neighbour))
                {
                    neighbour.gCost = costToNeighb;
                    neighbour.hCost = getDistance(graph, neighbour, goal_node);
                    neighbour.parent = current;

                    if (!open_set.Contains(neighbour))
                        open_set.Add(neighbour);
                    else
                    {
                        open_set.Remove(neighbour);
                        open_set.Add(neighbour);
                    }
                }

            }*/
            Debug.Log("----------------");
        }

    }

    public static float getDistance(Graph graph, Node start, Node end)
    {
        int x_distance = Math.Abs(start.i - end.i);
        int z_distance = Math.Abs(start.j - end.j);

        float diagonal_cost = (float)(Math.Sqrt(Math.Pow(Graph.x_unit,2) + Math.Pow(Graph.z_unit,2)));

        return diagonal_cost * (x_distance < z_distance ? x_distance : z_distance) + (x_distance >= z_distance ? x_distance * Graph.x_unit : z_distance * Graph.z_unit);

    }

    public static float[] getSteeringSteps(float maxSteering, int steps)
    {
        int size = 2 * steps + 1;
        float step = maxSteering / steps;
        float[] steeringSteps = new float[size];

        for (int i=0; i<size; i++)
        {
            steeringSteps[i] = -maxSteering + step * i;
        }
        return steeringSteps;
    }

    //gets the next coordinates and orientation given the velocity and the wheel steering angle based on the kinematic car model
    public static Vector4 nextCoordinates(float velocity, float steering_angle, float car_length, Vector4 position)
    {
        Debug.Log("Velocity and angle: " + velocity + " " + steering_angle);
        float theta = (float)(position.w + velocity / car_length * Math.Tan(steering_angle / 180 * Math.PI));

        Debug.Log("Original z: " + position.z + "Sin angle:" + theta + " Sin: " + Math.Sin(theta));
        float x = (float) (position.x + velocity * Math.Cos(theta));
        float z = (float)(position.z + velocity * Math.Sin(theta));
        ////TODODebug.Log("Starting from " + position + " and got to " + new Vector4(x, 0, z, w));
        return new Vector4(x, 0, z, theta);
    }

    private static Node getNextNode(Graph graph, Vector4 next_position)
    {
        float x = next_position.x;
        float z = next_position.z;
        Node node = graph.getNodeFromPoint(new Vector3(x, 0, z));
        ////TODODebug.Log("Next node: [" + node.i + "," + node.j + "]" + " position (" + node.x_pos + ", " + node.z_pos + ")");
        return node;
    }

    public static List<ContinousNode> getNextNodes(Graph graph, float car_length, Vector4 current_position, Tuple<float, float>[] combinations)
    {
        List<ContinousNode> toReturn = new List<ContinousNode>();
        for(int i=0; i<combinations.Length; i++)
        {
            Vector4 next_coordinates = nextCoordinates(combinations[i].Item1, combinations[i].Item2, car_length, current_position);
            ContinousNode next_node = new ContinousNode(getNextNode(graph, next_coordinates), next_coordinates);
            Debug.Log("Getting neigh [" + i + "] starting from position " + current_position + " with velocity" + combinations[i].Item1 + " and angle " + combinations[i].Item2+ " Next coord: " + next_coordinates + " node ["+ next_node.node.i +","+next_node.node.j+"]" + "node center:" + next_node.node.worldPosition);
            toReturn.Add(next_node);
        }
        return toReturn;
    }

    public static float newGetDistance(NewNode start, NewNode end) //euclidian distance between points
    {
        Vector3 start_pos = start.discrete_xyz_position;
        Vector3 end_pos = end.discrete_xyz_position;
        double x = start_pos.x - end_pos.x;
        double z = start_pos.z - end_pos.z;

        return (float) Math.Sqrt(x*x + z*z);

    }

    public static List<NewNode> NewGetNextNodes(float car_length, Vector4 current_position, Tuple<float, float>[] combinations)
    {
        List<NewNode> toReturn = new List<NewNode>();
        for (int i = 0; i < combinations.Length; i++)
        {
            Vector4 next_coordinates = nextCoordinates(combinations[i].Item1, combinations[i].Item2, car_length, current_position);

            NewNode next_node = NewNode.FindNode(next_coordinates);

            //Debug.Log("Getting neigh [" + i + "] starting from position " + current_position + " with velocity" + combinations[i].Item1 + " and angle " + combinations[i].Item2 + " Next coord: " + next_coordinates + " node [" + next_node.node.i + "," + next_node.node.j + "]" + "node center:" + next_node.node.worldPosition);
            toReturn.Add(next_node);
        }
        return toReturn;
    }

    public static List<NewNode> FindPath2(Graph graph, Vector4 start_position, Vector4 goal_position, float[] steering_angles, float max_additional_cost, float negative_bias)
    {
        NewNode startingNode = new NewNode(start_position);
        startingNode.additional_cost = 0;
        startingNode.gCost = 0;
        NewNode endingNode = new NewNode(goal_position);
        startingNode.hCost = newGetDistance(startingNode, endingNode);
        List<NewNode> path = new List<NewNode>();

        if (Graph.x_unit != 0)
        {
            float diagonal = (float)Math.Sqrt(Graph.x_unit * Graph.x_unit + Graph.z_unit * Graph.z_unit) + 0.01f;
            velocity[0] = -diagonal;
            velocity[1] = -diagonal / 2;
            velocity[2] = diagonal / 2;
            velocity[3] = diagonal;
        }

        int comb_size = steering_angles.Length * velocity.Length;
        Tuple<float, float>[] combinations = new Tuple<float, float>[comb_size];


        for (int i = 0; i < velocity.Length; i++)
            for (int j = 0; j < steering_angles.Length; j++)
            {
                //Debug.Log("DEBUG i = "+i+" j = "+j+" combinations index: " + (j + i * steering_angles.Length));
                combinations[j + i * steering_angles.Length] = new Tuple<float, float>(velocity[i], steering_angles[j]);
            }

        float[] additional_costs = {10,5,2.5f,5,10, //velocity = -diagonal
                                    5, 2.5f, 1.25f, 2.5f,5, //velocity = -diagonal/2
                                    1.25f, 0.75f, 0,  0.75f, 1.25f, //velocity = diagonal/2
                                    2.5f, 1.25f, 0.75f, 1.25f, 2.5f //velocity = diagonal
                                    };

        SortedSet<NewNode> open_set = new SortedSet<NewNode>(new NewNode.NodeComp());
        HashSet<NewNode> closed_set = new HashSet<NewNode>();


        open_set.Add(startingNode);

        NewNode current_node;

        int iteration_count = 10000;
        while (open_set.Count > 0)
        {
            iteration_count--;
            current_node = open_set.Min;
            newVisited_points.Add(current_node);

            open_set.Remove(current_node);
            closed_set.Add(current_node);

            if (current_node.isInGoalRange(endingNode.continous_position, 5f) || iteration_count == 0)
            {
                NewNode previous_node = current_node.parent;
                while(!previous_node.EqualsContinous(startingNode))
                {
                    path.Add(previous_node);
                    previous_node = previous_node.parent;
                }
                path.Add(previous_node);

                path.Reverse();
                return path;
            }

            List<NewNode> next_nodes = NewGetNextNodes(car_len, current_node.discrete_position, combinations);
            for(int i = 0; i<next_nodes.Count; i++) 
            {
                NewNode neighbour = next_nodes[i];
                
                if (!Graph.isWalkable(neighbour.continous_position)|| closed_set.Contains(neighbour))
                    continue;
                float costToNeighb = current_node.gCost + newGetDistance(current_node, neighbour) + additional_costs[i];
                if (!open_set.Contains(neighbour) || costToNeighb < neighbour.gCost) //if i've never considered the node or if i found a better path
                {
                    neighbour.gCost = costToNeighb;
                    neighbour.hCost = newGetDistance(neighbour, endingNode);
                    neighbour.parent = current_node;
                    if (!open_set.Contains(neighbour))
                        open_set.Add(neighbour);
                    else
                    {
                        open_set.Remove(neighbour);
                        open_set.Add(neighbour);
                    }
                }

            }

        }
        return null;
    }


}
