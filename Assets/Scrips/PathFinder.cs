using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class PathFinder : MonoBehaviour
{
    public static float[,] headings = new float[,] { { 135f, 90f, 45f }, { 180f, 0f, 0f }, { 225f, 270f, 315f } };

    public static void findPath(Graph graph, Vector3 start_position, Vector3 goal_position)
    {
        Node start_node = graph.getNodeFromPoint(start_position);
        Node goal_node = graph.getNodeFromPoint(goal_position);
        Debug.Log("Start i: " + start_node.i + "Start j:" + start_node.j);
        Debug.Log("Goal i: " + goal_node.i + "Goal j:" + goal_node.j);

        List<Node> open_set = new List<Node>();
        HashSet<Node> closed_set = new HashSet<Node>();
        open_set.Add(start_node);
        bool k = true;

        while (open_set.Count > 0)
        {

            Node current = open_set[0];
            if (k)
            {
                Debug.Log("Nodo fuori penalty: " + current.wallClosenessCost);
                Debug.Log("Totale: " + current.fCost);
            }
            for (int i=1; i<open_set.Count; i++)
            {
                if(open_set[i].fCost < current.fCost || open_set[i].fCost == current.fCost && open_set[i].hCost < current.hCost)
                {
                    current = open_set[i];
                }
            }
            open_set.Remove(current);
            closed_set.Add(current);
            if (current == goal_node)
            {
                List<Node> path = new List<Node>();
                Node previous_node = current;

                while (previous_node != start_node)
                {
                    //Debug.Log("Coordinates (" + previous_node.i + "," + previous_node.j + ")");
                    path.Add(previous_node);
                    previous_node = previous_node.parent;
                }
                path.Reverse();
                graph.path = path;
                foreach(Node n in graph.path)
                {
                    //Debug.Log("Path, nodo (" + previous_node.i + "," + previous_node.j + ")");
                }
            }

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
                    neighbour.heading = headings[-neighbour.j + current.j + 1, neighbour.i - current.i + 1];
                    String s = String.Format("Starting from [{0},{1}] to [{2},{3}] with angle {4}", current.i, current.j, neighbour.i, neighbour.j, neighbour.heading);
                    Debug.Log(s);
                    /*switch (current.heading){
                        case Node.Turn.A:
                            if (neighbour.i > current.i)
                                if(neighbour.j > current.j)
                                {
                                    neighbour.turn_from_parent[0] = Node.Turn.A;
                                    neighbour.turn_from_parent[1] = Node.Turn.R;
                                    neighbour.heading = 

                                }

                                else if (neighbour.i < current.i)
                            else neighbour.turn_from_parent[0] = neighbour.turn_from_parent[1];

                                        break;
                        case Node.Turn.B:
                            break;
                        case Node.Turn.L:
                            break;
                        case Node.Turn.R:
                            break;

                    }*/


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

    }

    public static float getDistance(Graph graph, Node start, Node end)
    {
        int x_distance = Math.Abs(start.i - end.i);
        int z_distance = Math.Abs(start.j - end.j);

        float diagonal_cost = (float)(Math.Sqrt(Math.Pow(graph.x_unit,2) + Math.Pow(graph.z_unit,2)));

        return diagonal_cost * (x_distance < z_distance ? x_distance : z_distance) + (x_distance >= z_distance ? x_distance * graph.x_unit : z_distance * graph.z_unit);

    }

}
