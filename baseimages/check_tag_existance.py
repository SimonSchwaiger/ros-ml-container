import json
import argparse

# This is a hack but is required to explicitly distinguish between ros 1 and ros 2 in the tags
# Noetic is an edgecase here
def image_lookup(distro):
    if distro=="noetic": return "ros1_noetic"
    else: return "ros2_{}".format(distro)

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Image Config Parser")
    parser.add_argument("json_file", help="Filepath to json file describing the image configs", type=str)
    parser.add_argument("ros_distro", help="ROS distribution to search for (e.g. humble)", type=str)
    parser.add_argument("graphics_platform", help="Graphics acceleration method", type=str)
    args = parser.parse_args()
    data = json.load(open(args.json_file))
    tags = [ entry["TAG"] for entry in data["include"] ]
    tag = "{}_{}".format(image_lookup(args.ros_distro), args.graphics_platform)
    # Lower case to be consistent with bash
    if tag in tags: print("true")
    else: print("false")