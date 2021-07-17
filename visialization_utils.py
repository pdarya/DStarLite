from sys import float_info

from PIL import Image, ImageDraw

EPS = float_info.epsilon


def get_image(grid_map, start_node=None, goal_node=None, traveled_path=None, planned_path=None, k=30):
    """
    Get PIL image for the agents state:
        - start, goal position
        - walls that are observed by the agent
        - path that is already travelled
        - planned path to the goal on the current agent position
    :param grid_map: Grid
    :param start_node: Node
    :param goal_node: Node
    :param traveled_path: list[tuple(int, int)]
    :param planned_path: list[tuple(int, int)]
    :return: PIL.Image
    """
    hIm = grid_map.height * k
    wIm = grid_map.width * k
    image = Image.new('RGB', (wIm, hIm), color='white')
    draw = ImageDraw.Draw(image)

    # draw grid with walls
    for row in range(grid_map.height):
        for column in range(grid_map.width):
            if grid_map.is_wall_cell_observed[row][column]:
                draw.rectangle((
                    column * k,
                    row * k,
                    (column + 1) * k - 1,
                    (row + 1) * k - 1
                ), fill=(70, 80, 80))

    # draw start and goal positions
    for node, fill_value in [(start_node, (40, 180, 99)), (goal_node, (231, 76, 60))]:
        if (node is not None) and (grid_map.is_traversable(node.row, node.column)):
            draw.rectangle((
                node.column * k,
                node.row * k,
                (node.column + 1) * k - 1,
                (node.row + 1) * k - 1
            ), fill=fill_value, width=0)

    # draw paths
    for path, color in [(traveled_path, 'green'), (planned_path, 'red')]:
        if len(path) > 1:
            scaled_path = [((column + 0.5) * k, (row + 0.5) * k) for row, column in path]
            draw.line(scaled_path, fill=color, width=3, joint='curve')

    # draw current position
    current_position = traveled_path[-1]
    draw.ellipse((
        current_position[1] * k,
        current_position[0] * k,
        (current_position[1] + 1) * k - 1,
        (current_position[0] + 1) * k - 1
    ), fill='blue', width=0)

    # draw observe range for current position
    draw.rectangle((
        (current_position[1] - 2) * k,
        (current_position[0] - 2) * k,
        (current_position[1] + 3) * k - 1,
        (current_position[0] + 3) * k - 1
    ), fill=None, width=3, outline='blue')

    return image
