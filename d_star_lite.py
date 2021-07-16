import numpy as np

from node import Node
from priority_queue import PriorityQueue
import utils


class DStarLiteRunner:
    def __init__(self, graph, start, goal, heuristic=utils.manhattan):
        self.graph = graph

        self.current_position = Node(position=start, g_value=np.inf, rhs_value=np.inf)
        self.goal = Node(position=goal, g_value=np.inf, rhs_value=0)
        self.heuristic = heuristic

        # maintain (row, column) -> node mapping
        self.nodes = {
            start: self.current_position,
            goal: self.goal,
        }
        self.k_m = 0

        self.queue = PriorityQueue()
        self.queue.add_node(self.goal, self.calculate_node_priority(self.goal))

    def calculate_node_priority(self, node):
        """
        :param node: Node
        :return: tuple(float, float)
            Node priority to possibly store it queue.
        """
        key_second = min(node.g_value, node.rhs_value)
        key_first = key_second + self.heuristic(node, self.current_position) + self.k_m
        return key_first, key_second

    def get_cost_by_neighbor(self, node, neighbour):
        """
        Calculates g-value for node given only one neighbor.

        :param node: Node
        :param neighbour: Node
        :return: float
        """
        return neighbour.g_value + self.graph.calculate_cost(neighbour, node)

    def get_lowest_cost_neighbour(self, node):
        """
        Get neighbor that minimizes g-value for the node.

        :param node: Node
        :return tuple(Node, float)
        """
        neighbors_positions = self.graph.get_neighbors(node.row, node.column)  # row, column
        neighbors_nodes = [self.nodes.get(position, Node(position)) for position in neighbors_positions]
        lowest_cost_neighbour = min(neighbors_nodes, key=lambda neighbor: self.get_cost_by_neighbor(node, neighbor))
        lowest_cost_neighbour_rhs = self.get_cost_by_neighbor(node, lowest_cost_neighbour)

        return lowest_cost_neighbour, lowest_cost_neighbour_rhs

    def calculate_rhs(self, node):
        """
        Calculate rhs-value for the node.

        :param node: Node
        :return: float
        """
        lowest_cost_neighbour, lowest_cost_neighbour_rhs = self.get_lowest_cost_neighbour(node)
        if lowest_cost_neighbour.position not in self.nodes:
            self.nodes[lowest_cost_neighbour.position] = lowest_cost_neighbour
        node.parent = lowest_cost_neighbour
        return lowest_cost_neighbour_rhs

    def update_node(self, node):
        if node != self.goal:
            node.rhs_value = self.calculate_rhs(node)

        self.queue.erase_node(node)

        if node.g_value != node.rhs_value:
            self.queue.add_node(node, self.calculate_node_priority(node))

    def get_neighbor_nodes(self, node):
        """
        Get neighbors for the node (even if it is wall).
        :param node: Node
        :return: list[Node]
        """
        neighbors_positions = self.graph.get_neighbors(node.row, node.column)  # (row, column)
        neighbors_nodes = self.get_nodes_by_position(neighbors_positions)
        return neighbors_nodes

    def update_nodes(self, nodes):
        for node in nodes:
            self.update_node(node)

    def compute_shortest_path(self):
        while self.queue.get_first_priority() < self.calculate_node_priority(self.current_position) or \
                self.current_position.rhs_value != self.current_position.g_value:
            k_old = self.queue.get_first_priority()
            current_node = self.queue.get_node()
            k_new = self.calculate_node_priority(current_node)

            if k_old < k_new:
                # print('k_old < k_new')
                self.queue.add_node(current_node, k_new)

            # overconsistent
            elif current_node.g_value > current_node.rhs_value:
                # print('overconsistent')
                current_node.g_value = current_node.rhs_value
                self.update_nodes(self.get_neighbor_nodes(current_node))

            # underconsistent
            else:
                # print('underconsistent')
                current_node.g_value = np.inf
                self.update_nodes(self.get_neighbor_nodes(current_node))
                self.update_node(current_node)

    def get_nodes_by_position(self, positions):
        """
        :param positions: iterable of (int, int)
        :return: list[Node]
        """
        nodes = []
        for position in positions:
            if position not in self.nodes:
                # adding nodes to mapping when we want to put them in a queue
                self.nodes[position] = Node(position)
            nodes.append(self.nodes[position])
        return nodes

    def get_path_to_goal(self, node):
        path = []
        current_node = node

        while current_node != self.goal:
            path.append(current_node.position)
            current_node, _ = self.get_lowest_cost_neighbour(current_node)
        path.append(current_node.position)
        return path

    def find_path(self):
        """
        Performs transitions towards the goal.
        """
        # updating graph
        self.graph.observe(self.current_position.row, self.current_position.column,
                           obs_range=2)
        last_node = self.current_position
        self.compute_shortest_path()

        yield self.current_position, len(self.nodes)

        while self.current_position != self.goal:
            if self.current_position.g_value == np.inf:
                raise Exception('Path not found.')

            # get neighbor with the lowest cost
            self.current_position, _ = self.get_lowest_cost_neighbour(self.current_position)
            yield self.current_position, len(self.nodes)

            new_walls_positions = self.graph.observe(
                self.current_position.row, self.current_position.column, obs_range=2,
            )
            if not new_walls_positions:
                print('No changes by moving to {}', self.current_position.position)
                continue

            self.k_m += self.heuristic(last_node, self.current_position)
            last_node = self.current_position

            # update new walls neighbors
            positions_to_update = set()
            for new_wall_position in new_walls_positions:
                neighbors_positions = self.graph.get_neighbors(*new_wall_position)
                for neighbor_position in neighbors_positions:
                    if self.graph.is_traversable(*neighbor_position):
                        # it is near wall and traversable by agents observations
                        positions_to_update.add(neighbor_position)
            self.update_nodes(self.get_nodes_by_position(positions_to_update))

            self.compute_shortest_path()
