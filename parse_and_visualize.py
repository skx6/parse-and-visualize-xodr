"""

"""


import os

from lxml import etree
from tqdm import tqdm

from opendriveparser import parse_opendrive
from math import pi, sin, cos


# Prepare the input file.
XODR_FILE = "data/test.xodr"


def to_color(r, g, b):
    return '#{:02x}{:02x}{:02x}'.format(r, g, b)


# Prepare the colors.
DRIVING_COLOR = (135, 151, 154)
TYPE_COLOR_DICT = {
    "shoulder": (136, 158, 131),
    "border": (84, 103, 80),
    "driving": DRIVING_COLOR,
    "stop": (128, 68, 59),
    "none": (236, 236, 236),
    "restricted": (165, 134, 88),
    "parking": DRIVING_COLOR,
    "median": (119, 155, 88),
    "biking": (108, 145, 125),
    "sidewalk": (106, 159, 170),
    "curb": (30, 49, 53),
    "exit": DRIVING_COLOR,
    "entry": DRIVING_COLOR,
    "onramp": DRIVING_COLOR,
    "offRamp": DRIVING_COLOR,
    "connectingRamp": DRIVING_COLOR,
    "onRamp": DRIVING_COLOR,
    "bidirectional": DRIVING_COLOR,
}
TYPE_COLOR_DICT = {k: to_color(*v) for k, v in TYPE_COLOR_DICT.items()}
COLOR_CENTER_LANE = "#FFC500"
COLOR_REFERECE_LINE = "#0000EE"

# Prepare sample step.
STEP = 0.1


def load_xodr_and_parse(file=XODR_FILE):
    """
    Load and parse .xodr file.
    :param file:
    :return:
    """
    with open(file, 'r') as fh:
        parser = etree.XMLParser()
        root_node = etree.parse(fh, parser).getroot()
        road_network = parse_opendrive(root_node)
    return road_network


def calculate_reference_points_of_one_geometry(geometry, length, step=0.01):
    """
    Calculate the stepwise reference points with position(x, y), tangent and distance between the point and the start.
    :param geometry:
    :param length:
    :param step:
    :return:
    """
    nums = int(length / step)
    res = []
    for i in range(nums):
        s_ = step * i
        pos_, tangent_ = geometry.calcPosition(s_)
        x, y = pos_
        one_point = {
            "position": (x, y),     # The location of the reference point
            "tangent": tangent_,    # Orientation of the reference point
            "s_geometry": s_,       # The distance between the start point of the geometry and current point along the reference line
        }
        res.append(one_point)
    return res


def get_geometry_length(geometry):
    """
    Get the length of one geometry (or the length of the reference line of the geometry).
    :param geometry:
    :return:
    """
    if hasattr(geometry, "length"):
        length = geometry.length
    elif hasattr(geometry, "_length"):
        length = geometry._length           # Some geometry has the attribute "_length".
    else:
        raise AttributeError("No attribute length found!!!")
    return length


def get_all_reference_points_of_one_road(geometries, step=0.01):
    """
    Obtain the sampling point of the reference line of the road, including:
    the position of the point
    the direction of the reference line at the point
    the distance of the point along the reference line relative to the start of the road
    the distance of the point relative to the start of geometry along the reference line
    :param geometries: Geometries of one road.
    :param step: Calculate steps.
    :return:
    """
    reference_points = []
    s_start_road = 0
    for geometry_id, geometry in enumerate(geometries):
        geometry_length = get_geometry_length(geometry)

        # Calculate all the reference points of current geometry.
        pos_tangent_s_list = calculate_reference_points_of_one_geometry(geometry, geometry_length, step=step)

        # As for every reference points, add the distance start by road and its geometry index.
        pos_tangent_s_s_list = [{**point,
                                 "s_road": point["s_geometry"]+s_start_road,
                                 "index_geometry": geometry_id}
                                for point in pos_tangent_s_list]
        reference_points.extend(pos_tangent_s_s_list)

        s_start_road += geometry_length
    return reference_points


def get_width(widths, s):
    assert isinstance(widths, list), TypeError(type(widths))
    widths.sort(key=lambda x: x.sOffset)
    current_width = None
    # EPS = 1e-5
    milestones = [width.sOffset for width in widths] + [float("inf")]

    control_mini_section = [(start, end) for (start, end) in zip(milestones[:-1], milestones[1:])]
    for width, start_end in zip(widths, control_mini_section):
        start, end = start_end
        if start <= s < end:
            ds = s - width.sOffset
            current_width = width.a + width.b * ds + width.c * ds ** 2 + width.d * ds ** 3
    return current_width


def get_lane_offset(lane_offsets, section_s, length=float("inf")):

    assert isinstance(lane_offsets, list), TypeError(type(lane_offsets))
    if not lane_offsets:
        return 0
    lane_offsets.sort(key=lambda x: x.sPos)
    current_offset = 0
    EPS = 1e-5
    milestones = [lane_offset.sPos for lane_offset in lane_offsets] + [length+EPS]

    control_mini_section = [(start, end) for (start, end) in zip(milestones[:-1], milestones[1:])]
    for offset_params, start_end in zip(lane_offsets, control_mini_section):
        start, end = start_end
        if start <= section_s < end:
            ds = section_s - offset_params.sPos
            current_offset = offset_params.a + offset_params.b * ds + offset_params.c * ds ** 2 + offset_params.d * ds ** 3
    return current_offset


class LaneOffsetCalculate:

    def __init__(self, lane_offsets):
        lane_offsets = list(sorted(lane_offsets, key=lambda x: x.sPos))
        lane_offsets_dict = dict()
        for lane_offset in lane_offsets:
            a = lane_offset.a
            b = lane_offset.b
            c = lane_offset.c
            d = lane_offset.d
            s_start = lane_offset.sPos
            lane_offsets_dict[s_start] = (a, b, c, d)
        self.lane_offsets_dict = lane_offsets_dict

    def calculate_offset(self, s):
        for s_start, (a, b, c, d) in reversed(self.lane_offsets_dict.items()): # e.g. 75, 25
            if s >= s_start:
                ds = s - s_start
                offset = a + b * ds + c * ds ** 2 + d * ds ** 3
                return offset
        return 0


def calculate_area_of_one_left_lane(left_lane, points, most_left_points):
    inner_points = most_left_points[:]

    widths = left_lane.widths
    update_points = []
    for reference_point, inner_point in zip(points, inner_points):

        tangent = reference_point["tangent"]
        s_lane_section = reference_point["s_lane_section"]
        lane_width = get_width(widths, s_lane_section)

        normal_left = tangent + pi / 2
        x_inner, y_inner = inner_point

        lane_width_offset = lane_width

        x_outer = x_inner + cos(normal_left) * lane_width_offset
        y_outer = y_inner + sin(normal_left) * lane_width_offset

        update_points.append((x_outer, y_outer))

    outer_points = update_points[:]
    most_left_points = outer_points[:]

    current_ara = {
        "inner": inner_points,
        "outer": outer_points,
    }
    return current_ara, most_left_points


def calculate_area_of_one_right_lane(right_lane, points, most_right_points):
    inner_points = most_right_points[:]

    widths = right_lane.widths
    update_points = []
    for reference_point, inner_point in zip(points, inner_points):

        tangent = reference_point["tangent"]
        s_lane_section = reference_point["s_lane_section"]
        lane_width = get_width(widths, s_lane_section)

        normal_eight = tangent - pi / 2
        x_inner, y_inner = inner_point

        lane_width_offset = lane_width

        x_outer = x_inner + cos(normal_eight) * lane_width_offset
        y_outer = y_inner + sin(normal_eight) * lane_width_offset

        update_points.append((x_outer, y_outer))

    outer_points = update_points[:]
    most_right_points = outer_points[:]

    current_ara = {
        "inner": inner_points,
        "outer": outer_points,
    }
    return current_ara, most_right_points


def calculate_lane_area_within_one_lane_section(lane_section, points):
    """
    Lane areas are represented by boundary lattice. Calculate boundary points of every lanes.
    :param lane_section:
    :param points:
    :return:
    """

    all_lanes = lane_section.allLanes

    # Process the lane indexes.
    left_lanes = [lane for lane in all_lanes if int(lane.id) > 0]
    right_lanes = [lane for lane in all_lanes if int(lane.id) < 0]
    left_lanes.sort(key=lambda x: x.id)
    right_lanes.sort(reverse=True, key=lambda x: x.id)

    # Get the lane area of left lanes and the most left lane line.
    left_lanes_area = dict()
    most_left_points = [point["position_center_lane"] for point in points][:]
    for left_lane in left_lanes:
        current_area, most_left_points = calculate_area_of_one_left_lane(left_lane, points, most_left_points)
        left_lanes_area[left_lane.id] = current_area

    # Get the lane area of right lanes and the most right lane line.
    right_lanes_area = dict()
    most_right_points = [point["position_center_lane"] for point in points][:]
    for right_lane in right_lanes:
        current_area, most_right_points = calculate_area_of_one_right_lane(right_lane, points, most_right_points)
        right_lanes_area[right_lane.id] = current_area

    return left_lanes_area, right_lanes_area, most_left_points, most_right_points


def calculate_points_of_reference_line_of_one_section(points):
    """
    Calculate center lane points accoding to the reference points and offsets.
    :param points: Points on reference line including position and tangent.
    :return: Updated points.
    """
    res = []
    for point in points:
        tangent = point["tangent"]
        x, y = point["position"]    # Points on reference line.
        normal = tangent + pi / 2
        lane_offset = point["lane_offset"]  # Offset of center lane.

        x += cos(normal) * lane_offset
        y += sin(normal) * lane_offset

        point = {
            **point,
            "position_center_lane": (x, y),
        }
        res.append(point)
    return res


def calculate_s_lane_section(reference_points, lane_sections):

    res = []
    for point in reference_points:

        for lane_section in reversed(lane_sections):
            if point["s_road"] >= lane_section.sPos:
                res.append(
                    {
                        **point,
                        "s_lane_section": point["s_road"] - lane_section.sPos,
                        "index_lane_section": lane_section.idx,
                    }
                )
                break
    return res


def uncompress_dict_list(dict_list: list):
    assert isinstance(dict_list, list), TypeError("Keys")
    if not dict_list:
        return dict()

    keys = set(dict_list[0].keys())
    for dct in dict_list:
        cur = set(dct.keys())
        assert keys == cur, "Inconsistency of dict keys! {} {}".format(keys, cur)

    res = dict()
    for sample in dict_list:
        for k, v in sample.items():
            if k not in res:
                res[k] = [v]
            else:
                res[k].append(v)

    keys = list(sorted(list(keys)))
    res = {k: res[k] for k in keys}
    return res


def get_lane_line(section_data: dict):
    """
    提取车道分界线
    :param section_data:
    :return:
    """
    left_lanes_area = section_data["left_lanes_area"]
    right_lanes_area = section_data["right_lanes_area"]

    lane_line_left = dict()
    if left_lanes_area:
        indexes = list(left_lanes_area.keys())  # 默认是排好序的
        for index_inner, index_outer in zip(indexes, indexes[1:] + ["NAN"]):
            lane_line_left[(index_inner, index_outer)] = left_lanes_area[index_inner]["outer"]

    lane_line_right = dict()
    if right_lanes_area:
        indexes = list(right_lanes_area.keys())  # 默认是排好序的
        for index_inner, index_outer in zip(indexes, indexes[1:] + ["NAN"]):
            lane_line_right[(index_inner, index_outer)] = right_lanes_area[index_inner]["outer"]

    return {"lane_line_left": lane_line_left, "lane_line_right": lane_line_right}


def get_lane_area_of_one_road(road, step=0.01):
    """
    Get all corresponding positions of every lane section in one road.
    :param road:
    :param step:
    :return: A dictionary of dictionary: {(road id, lane section id): section data}
    Section data is a dictionary of position information.
    section_data = {
        "left_lanes_area": left_lanes_area,
        "right_lanes_area": right_lanes_area,
        "most_left_points": most_left_points,
        "most_right_points": most_right_points,
        "types": types,
        "reference_points": uncompressed_lane_section_data,
    }
    """
    geometries = road.planView._geometries
    # Lane offset is the offset between center lane (width is 0) and the reference line.
    lane_offsets = road.lanes.laneOffsets
    lane_offset_calculate = LaneOffsetCalculate(lane_offsets=lane_offsets)
    lane_sections = road.lanes.laneSections
    lane_sections = list(sorted(lane_sections, key=lambda x: x.sPos))   # Sort the lane sections by start position.

    reference_points = get_all_reference_points_of_one_road(geometries, step=step)  # Extract the reference points.

    # Calculate the offsets of center lane.
    reference_points = [{**point, "lane_offset":  lane_offset_calculate.calculate_offset(point["s_road"])}
                        for point in reference_points]

    # Calculate the points of center lane based on reference points and offsets.
    reference_points = calculate_points_of_reference_line_of_one_section(reference_points)

    # Calculate the distance of each point starting from the current section along the direction of the reference line.
    reference_points = calculate_s_lane_section(reference_points, lane_sections)

    total_areas = dict()
    for lane_section in lane_sections:
        section_start = lane_section.sPos  # Start position of the section in current road.
        section_end = lane_section.sPos + lane_section.length  # End position of the section in current road.

        # Filter out the points belonging to current lane section.
        current_reference_points = list(filter(lambda x: section_start <= x["s_road"] < section_end, reference_points))

        # Calculate the boundary point of every lane in current lane section.
        area = calculate_lane_area_within_one_lane_section(lane_section, current_reference_points)
        left_lanes_area, right_lanes_area, most_left_points, most_right_points = area

        # Extract types and indexes.
        types = {lane.id: lane.type for lane in lane_section.allLanes if lane.id != 0}
        index = (road.id, lane_section.idx)

        # Convert dict list to list dict of the reference points information.
        uncompressed_lane_section_data = uncompress_dict_list(current_reference_points)

        # Integrate all the information of current lane section of current road.
        section_data = {
            "left_lanes_area": left_lanes_area,
            "right_lanes_area": right_lanes_area,
            "most_left_points": most_left_points,
            "most_right_points": most_right_points,
            "types": types,
            "reference_points": uncompressed_lane_section_data,  # 这些是lane section的信息
        }

        # Get all lane lines with their left and right lanes.
        lane_line = get_lane_line(section_data)
        section_data.update(lane_line)

        total_areas[index] = section_data

    return total_areas


def get_all_lanes(road_network, step=0.1):
    """
    Get all lanes of one road network.
    :param road_network: Parsed road network.
    :param step: Step of calculation.
    :return: Dictionary with the following format:
        keys: (road id, lane section id)
        values: dict(left_lanes_area, right_lanes_area, most_left_points, most_right_points, types, reference_points)
    """
    roads = road_network.roads
    total_areas_all_roads = dict()

    for road in tqdm(roads, desc="Calculating boundary points."):
        lanes_of_one_road = get_lane_area_of_one_road(road, step=step)
        total_areas_all_roads = {**total_areas_all_roads, **lanes_of_one_road}

    return total_areas_all_roads


def rescale_color(hex_color, rate=0.5):
    """
    Half the light of input color, e.g. white => grey.
    :param hex_color: e.g. #a55f13
    :param rate: Scale rate from 0 to 1.
    :return:
    """

    r = int(hex_color[1:3], 16)
    g = int(hex_color[3:5], 16)
    b = int(hex_color[5:7], 16)

    # Half the colors.
    r = min(255, max(0, int(r * rate)))
    g = min(255, max(0, int(g * rate)))
    b = min(255, max(0, int(b * rate)))

    r = hex(r)[2:]
    g = hex(g)[2:]
    b = hex(b)[2:]

    r = r.rjust(2, "0")
    g = g.rjust(2, "0")
    b = b.rjust(2, "0")

    new_hex_color = '#{}{}{}'.format(r, g, b)
    return new_hex_color


def plot_planes_of_roads(total_areas, save_folder):
    """
    Plot the roads.
    :param total_areas:
    :param save_folder:
    :return:
    """

    import matplotlib.pyplot as plt
    plt.cla()

    plt.figure(figsize=(160, 90))
    area_select = 10  # select one from 10 boundary points for accelerating.

    all_types = set()

    # Plot lane area.
    for k, v in tqdm(total_areas.items(), desc="Ploting Roads"):
        left_lanes_area = v["left_lanes_area"]
        right_lanes_area = v["right_lanes_area"]

        types = v["types"]

        for left_lane_id, left_lane_area in left_lanes_area.items():
            type_of_lane = types[left_lane_id]
            all_types.add(type_of_lane)
            lane_color = TYPE_COLOR_DICT[type_of_lane]
            inner_points = left_lane_area["inner"]
            outer_points = left_lane_area["outer"]

            points_of_one_road = inner_points + outer_points[::-1]
            xs = [i for i, _ in points_of_one_road]
            ys = [i for _, i in points_of_one_road]
            plt.fill(xs, ys, color=lane_color, label=type_of_lane)
            plt.scatter(xs[::area_select], ys[::area_select], color=rescale_color(lane_color, 0.5), s=1)

        for right_lane_id, right_lane_area in right_lanes_area.items():
            type_of_lane = types[right_lane_id]
            all_types.add(type_of_lane)

            lane_color = TYPE_COLOR_DICT[type_of_lane]
            inner_points = right_lane_area["inner"]
            outer_points = right_lane_area["outer"]

            points_of_one_road = inner_points + outer_points[::-1]
            xs = [i for i, _ in points_of_one_road]
            ys = [i for _, i in points_of_one_road]
            plt.fill(xs, ys, color=lane_color, label=type_of_lane)
            plt.scatter(xs[::area_select], ys[::area_select], color=rescale_color(lane_color, 0.5), s=1)

    # Plot boundaries
    for k, v in tqdm(total_areas.items(), desc="Ploting Edges"):
        left_lanes_area = v["left_lanes_area"]
        right_lanes_area = v["right_lanes_area"]

        types = v["types"]
        for left_lane_id, left_lane_area in left_lanes_area.items():
            type_of_lane = types[left_lane_id]
            all_types.add(type_of_lane)
            lane_color = TYPE_COLOR_DICT[type_of_lane]
            inner_points = left_lane_area["inner"]
            outer_points = left_lane_area["outer"]
            points_of_one_road = inner_points + outer_points[::-1]
            xs = [i for i, _ in points_of_one_road]
            ys = [i for _, i in points_of_one_road]
            plt.scatter(xs[::area_select], ys[::area_select], color=rescale_color(lane_color, 0.5), s=1)

        for right_lane_id, right_lane_area in right_lanes_area.items():
            type_of_lane = types[right_lane_id]
            all_types.add(type_of_lane)
            lane_color = TYPE_COLOR_DICT[type_of_lane]
            inner_points = right_lane_area["inner"]
            outer_points = right_lane_area["outer"]
            points_of_one_road = inner_points + outer_points[::-1]
            xs = [i for i, _ in points_of_one_road]
            ys = [i for _, i in points_of_one_road]
            plt.scatter(xs[::area_select], ys[::area_select], color=rescale_color(lane_color, 0.5), s=1)

    # Plot center lane and reference line.
    saved_ceter_lanes = dict()
    for k, v in tqdm(total_areas.items(), desc="Ploting Reference and center"):

        reference_points = v["reference_points"]
        if not reference_points:
            continue
        position_reference_points = reference_points["position"]
        position_center_lane = reference_points["position_center_lane"]

        position_reference_points_xs = [x for x, y in position_reference_points]
        position_reference_points_ys = [y for x, y in position_reference_points]
        position_center_lane_xs = [x for x, y in position_center_lane]
        position_center_lane_ys = [y for x, y in position_center_lane]

        saved_ceter_lanes[k] = position_center_lane
        plt.scatter(position_reference_points_xs, position_reference_points_ys, color=COLOR_REFERECE_LINE, s=3)
        plt.scatter(position_center_lane_xs, position_center_lane_ys, color=COLOR_CENTER_LANE, s=2)

    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    # Create legend.
    legend_dict = {
        # k: Patch(facecolor=v, edgecolor=v, alpha=0.3) for k, v in type_color_dict.items() if k in all_types
        k: Patch(facecolor=v, edgecolor=v, alpha=1.0) for k, v in TYPE_COLOR_DICT.items() if k in all_types
    }
    legend_dict.update({
        "center_lane": Patch(facecolor=COLOR_CENTER_LANE, edgecolor=COLOR_CENTER_LANE, alpha=1.0),
    })
    legend_dict.update({
        "reference_line": Patch(facecolor=COLOR_REFERECE_LINE, edgecolor=COLOR_REFERECE_LINE, alpha=1.0),
    })

    plt.legend(handles=legend_dict.values(), labels=legend_dict.keys(), fontsize=50)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")

    os.makedirs(save_folder, exist_ok=True)
    save_pdf_file = os.path.join(save_folder, "lanes.pdf")
    plt.savefig(save_pdf_file)


def process_one_file(file, step=0.1):
    """
    Load one .xodr file and calculate the railing positions with other important messages.
    :param file: Input file.
    :param step: Step of calculation.
    :return: None
    """

    assert os.path.exists(file), FileNotFoundError(file)
    d, ne = os.path.split(file)
    n, e = os.path.splitext(ne)
    save_folder = os.path.join(d, n)

    road_network = load_xodr_and_parse(file)
    total_areas = get_all_lanes(road_network, step=step)

    plot_planes_of_roads(total_areas, save_folder)


def main():
    process_one_file(file=XODR_FILE)


if __name__ == "__main__":
    main()
