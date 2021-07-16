def manhattan(node_from, node_to):
    return abs(node_from.row - node_to.row) + abs(node_from.column - node_to.column)
