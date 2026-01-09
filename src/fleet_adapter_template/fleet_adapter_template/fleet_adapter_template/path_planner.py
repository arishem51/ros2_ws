from collections import defaultdict, deque
import logging

logger = logging.getLogger(__name__)

class PathPlanner:
    def __init__(self):
        pass
    
    def _expand_path(self, start, goal, graph: defaultdict[str, list[str]]):
        """
        Find a path from start node to goal node using BFS.
        
        Args:
            start: Starting node name
            goal: Goal node name
            graph: Graph represented as adjacency list
            
        Returns:
            List of node names representing the path, or None if no path exists
        """
        queue = deque([[start]])
        visited = set([start])
        while queue:
            path = queue.popleft()
            current = path[-1]
            if current == goal:
                return path
            for neighbor in graph[current]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(path + [neighbor])
        return None
    
    def calculate_path(self, start_node_name: str, goal_node_name: str, graph: defaultdict[str, list[str]]):
        """
        Calculate path between two nodes.
        
        Args:
            start_node_name: Name of the starting node
            goal_node_name: Name of the goal node
            graph: Graph represented as adjacency list
            
        Returns:
            List of node names representing the path, or None if no path exists
        """
        if start_node_name is None or goal_node_name is None:
            return None
        
        path = self._expand_path(start_node_name, goal_node_name, graph)
        logger.info(f"Path from {start_node_name} to {goal_node_name}: {path}")
        
        return path
