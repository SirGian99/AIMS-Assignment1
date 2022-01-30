using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class PathFinder : MonoBehaviour
{

    public static void findPath(Graph graph, Vector3 start_position, Vector3 goal_position)
    {
        Node start_node = graph.getNodeFromPoint(start_position);
        Node goal_node = graph.getNodeFromPoint(goal_position);
        Debug.Log("Start i: " + start_node.i + "Start j:" + start_node.j);
        Debug.Log("Goal i: " + goal_node.i + "Goal j:" + goal_node.j);

        List<Node> open_set = new List<Node>();
        HashSet<Node> closed_set = new HashSet<Node>();
        open_set.Add(start_node);

        while (open_set.Count > 0)
        {
            //Debug.Log("Sono nel while");
            Node current = open_set[0];
            for(int i=1; i<open_set.Count; i++)
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
                    Debug.Log("Path, nodo (" + previous_node.i + "," + previous_node.j + ")");
                }
            }

            foreach(Node neighbour in graph.getNeighbours(current))
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
                }

            }
        }

    }

    public static float getDistance(Graph graph, Node start, Node end)
    {
        int x_distance = Math.Abs(start.i - end.i);
        int z_distance = Math.Abs(start.j - end.j);

        int diagonal_cost = 14;
        int straight_cost = 10; //TODO MODIFY THEM

        return diagonal_cost * (x_distance < z_distance ? x_distance : z_distance) + straight_cost * (x_distance * graph.x_unit + z_distance * graph.z_unit);

    }

}
