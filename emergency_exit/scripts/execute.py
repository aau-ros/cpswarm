#!/usr/bin/python
import os
import argparse
import rospkg
import time
import ConfigParser
import socket

prolog = ""
prolog += "Create and run multi-robot simulation.\n\n"
prolog += "Select an execution mode for the script."

epilog = "Enjoy and have fun!"
parser = argparse.ArgumentParser(description=prolog, epilog=epilog,formatter_class=argparse.RawTextHelpFormatter)

execution_group = parser.add_mutually_exclusive_group(required=False)

# EXECUTION MODES
execution_group.add_argument("--simulate-exploration",dest='simulate_exploration',action="store_true",
                    help="Runs the multi-robot exploration in simulation",
                    default=False)
execution_group.add_argument("--simulate-communication",dest="simulate_communication",
                    action="store_true", default=False,
                    help="Only starts communication capabilities in simulation mode.")
execution_group.add_argument("--test-communication",dest="test_communication",
                    action="store_true",default=False,
                    help="Starts only communication nodes (no simulation!).")


# GENERAL SETTINGS
parser.add_argument("--map-size", default=[35,35], type=int, nargs=2,help="The size of the map", dest="map_size")
parser.add_argument("--screen-output",dest="screen_output",default=False,help="Set ROS output to screen.",
		    action="store_true")
parser.add_argument("--exp-strategy",dest="exploration_strategy",type=int,nargs="?",default=6,
		    help="The strategy to be used for exploration.",choices=range(0,7))
parser.add_argument("--interface",dest='interface',nargs='?',type=str,default='wlan0')
parser.add_argument("--connectivity",default=[],type=str,nargs="+",
                    help="Loads white- or blacklist including which robot can communication with whom.")


# MEASUREMENT SETTINGS
parser.add_argument("--profile",const="valgrind --tool=callgrind --log-file=./adhoc_%%p.callgrind",
                    help="Uses valgrind to profile the adhoc_comm node", nargs="?",
                    dest="profile")
parser.add_argument("--traffic-generator",dest="traffic_generator",nargs="?",type=str,
		    help="Loads the traffic generator node.", const="default.cfg",default=False)
parser.add_argument("--traffic-receiver",dest="traffic_receiver",action="store_true",default=False,
		    help="Starts the traffic receiver but not the generator.")


# SIMULATION SETTINGS
parser.add_argument("-n", "--number-of-robots", dest="num_robots", type=int,
                    help="Number of robots to simulate", metavar="N", default=2,
                   nargs="?")
parser.add_argument("-i", "--sim-id", default=1, nargs="?", metavar="ID",
                    help="The ID of the simulation run", dest="sim_id")
parser.add_argument("-t", "--time", metavar="HH-MM-SS", default=[time.strftime("%H-%M-%S", time.localtime())],
                    dest="time", nargs=1, help="The name of the 'time'-directory for the log path.")
parser.add_argument("-m", "--map", default="small_map.png", type=str, nargs="?", metavar="MAP.png",
                    help="The map to be used in the world file.", dest="map")
parser.add_argument("--gui",dest="gui", action="store_true", default=False,
                    help="Enable stage GUI")
parser.add_argument("-d", "--date", metavar="yy-mm-dd", default=[time.strftime("%y-%m-%d", time.localtime())],
                    dest="date", nargs=1, help="The name of the 'time'-directory for the log path.")
parser.add_argument("--emulate-comm", action="store_true", default=False,
                    help="Emulates reliable multicast transmission using the adhoc_communication_emulator node.",
                    dest="emulate_comm")
parser.add_argument("--list-maps",dest="list_maps",default=False,action="store_true",
                    help="List available maps")
parser.add_argument("--robot-id",const=1,type=int,dest="robot_id",nargs="?",
                    help="Sets the robot ID and only starts a single robot with the given Robot ID")

args, unknown = parser.parse_known_args()








###############################################################################
# SETTINGS & VARIABLES & FUNCTIONS
###############################################################################

def list_maps():
    print "Available (png) maps:"
    for map in sorted(os.listdir(os.path.join(package_path, world_dir))):
        if map.endswith(".png"):
            print map

rospack = rospkg.RosPack()
package_path = rospack.get_path("multi_robot_simulation")

launch_file = "auto_generated.launch";
launch_path = os.path.join(package_path, "launch", launch_file)
world_dir = "world"
world_file = "world.world"
world_path = os.path.join(package_path,world_dir,world_file)
map_file = args.map
map_path = os.path.join(package_path, world_dir, map_file)
sim_id = args.sim_id
colors = ["red", "blue", "green", "silver"]
adhoc_connectivity_path = os.path.join(package_path, "adhoc_connectivity")

t = time.localtime()
log_path = os.path.join(rospack.get_path("multi_robot_analyzer"), "logs", args.date[0], args.time[0], str(args.sim_id))

if args.simulate_communication or args.simulate_exploration:
    simulation = True
else:
    simulation = False
    args.num_robots = 1

if args.screen_output:
    output_type = "screen"
else:
    output_type = "log"









###############################################################################
# CHECK PARAMETERS
###############################################################################
if args.list_maps:
    list_maps()
    exit(0)

if not (args.simulate_communication or args.simulate_exploration or args.test_communication):
    print "Please specify execution mode by selecting either"
    print "   --simulate-communicaition"
    print "   --simulate-exploration"
    print "   --test-communication"
    exit(0)


if len(unknown) > 0:
    print "error: unrecognized arguments: %s" %unknown
    print "Please check the new syntax of this script.\n"
    parser.print_help()
    exit(0)

if args.robot_id and args.robot_id >= args.num_robots:
    print "error: invalid robot ID '%d' bigger than number of robots '%d'" %(args.robot_id,args.num_robots)
    exit()


if not args.simulate_communication and not os.path.isfile(map_path):
    print "Cannot find map '%s' in directory '%s'. Aborting." %(map_file, world_path)
    list_maps()
    exit(1)
else:
    print "Found map."



print("Starting simulation with parameters:")
print("\tpackage path  :  %s" %package_path)
print("\tlaunch_file   :  %s" %launch_path)
print("\tlog_pah       :  %s" %log_path)
print("\tconnectivity  :  %s" %adhoc_connectivity_path)
if simulation:
    print("\tnum_robots    :  %s" %args.num_robots)
    print("\tsim_id        :  %s" %args.sim_id )
    print("\tworld_file    :  %s" %world_path)
    print("\tmap_path      :  %s" %map_path)
    print("\trobot_id      :  %s" %args.robot_id)


if simulation or args.connectivity:
    robot_macs = {}
    connected_macs = {}

if simulation:
    # determine all mac_addresses for the robots of the form
    # robot_name,mac_address. Multiple robots are separated by '!'
    # This is required fot the adhoc_communication node to work properly when using
    # stage.
    all_macs = ""
    for num_robot in range(args.num_robots):
        robot_name = "robot_" + str(num_robot)
        robot_macs[robot_name] = "02:%02d:00:00:00:00" %(num_robot + 1)
        if all_macs:
            all_macs += "!"
        all_macs += "%s,%s" %(robot_name,robot_macs[robot_name])
    # per default all direct connection between all robots
    for robot_name in robot_macs.keys():
        connected_macs[robot_name] = all_macs


if args.connectivity:
    # Set the connectivity between robots for tests
    for f in args.connectivity:
        conn_file = os.path.join(adhoc_connectivity_path, f)
        if not os.path.isfile(conn_file):
            print "ERROR: Cannot find file '%s'. Aborting." %conn_file
            print "Possible files are:"
            for f in os.listdir(adhoc_connectivity_path):
                print "- %s" %f
            exit(0)
        fh = open(conn_file)
        lines = fh.readlines()
        for line in lines:
            line = line.strip()
            if line.startswith("#") or not line:
                continue
            line = line.strip()
            robot_name, conns = line.split(";")
            #print "robot:", robot_name, "conns", conns
            if simulation and not ":" in conns:
                # MAC addresses not specified for simulation; self generate
                conn_str = ""
                for robot in conns.split(","):
                    if robot:
                        robot += ",%s" %robot_macs[robot]
                        if conn_str:
                            conn_str += "!"
                        conn_str += robot
                connected_macs[robot_name] = conn_str
            else:
                # use the MAC addresses specified in the whitelist
                connected_macs[robot_name] = conns






###############################################################################
# MAP PARAMETERS
###############################################################################

# robot positions according to map
position = {}          # 1. value: must be zero
                       # 2. value: rotation of robot in degrees:
                       #           0:   facing right
                       #           90:  facing up
                       #           180: facing left
                       #           270: facing down
                       #           in rviz robot is always facing right, map is rotated instead
position_x = {}        # x coordinate of robot relative to origin in stage coordinates (positive to right)
position_y = {}        # y coordinate of robot relative to origin in stage coordinates (positive upwards)
position_x_offset = {} # horizontal distance between robots in stage coordinates (meters)
position_y_offset = {} # vertical distance between robots in stage coordinates (meters)
size = {}              # 1. value: width of map in stage coordinates (meters), this defines the map resolution!
                       # 2. value: height of map in stage coordinates (meters), this defines the map resolution!
                       # 3. value: changes nothing, cannot be zero
                       # the size defines the resolution as it changes the map size relative to the robot size


position["oedk.png"] = "0 90.0"
position_x["oedk.png"] = 0
position_y["oedk.png"] = 0
position_x_offset["oedk.png"] = 1
position_y_offset["oedk.png"] = 0
size["oedk.png"] = "[103.74 106.68 0.4]"

position["vienna.png"] = "0 0.0"
position_x["vienna.png"] = 4
position_y["vienna.png"] = -8
position_x_offset["vienna.png"] = 0
position_y_offset["vienna.png"] = 0
size["vienna.png"] = "[350 178.45 0.4]"

position["vienna_large.png"] = "0 0.0"
position_x["vienna_large.png"] = 12
position_y["vienna_large.png"] = -24
position_x_offset["vienna_large.png"] = 0
position_y_offset["vienna_large.png"] = 0
size["vienna_large.png"] = "[1000 510 0.4]"

position["manhattan.png"] = "0 0.0"
position_x["manhattan.png"] = 0
position_y["manhattan.png"] = 0
position_x_offset["manhattan.png"] = 0
position_y_offset["manhattan.png"] = 0
size["manhattan.png"] = "[446.4 346.4 0.4]"

position["manhattan_2.png"] = "0 0.0"
position_x["manhattan_2.png"] = 0
position_y["manhattan_2.png"] = 0
position_x_offset["manhattan_2.png"] = 0
position_y_offset["manhattan_2.png"] = 0
size["manhattan_2.png"] = "[363.9 363.9 0.4]"

position["manhattan_3.png"] = "0 0.0"
position_x["manhattan_3.png"] = 0
position_y["manhattan_3.png"] = 0
position_x_offset["manhattan_3.png"] = 0
position_y_offset["manhattan_3.png"] = 0
size["manhattan_3.png"] = "[292.5 292.5 0.4]"

position["manhattan_5.png"] = "0 0.0"
position_x["manhattan_5.png"] = 0
position_y["manhattan_5.png"] = 0
position_x_offset["manhattan_5.png"] = 0
position_y_offset["manhattan_5.png"] = 0
size["manhattan_5.png"] = "[1495.5 1495.5 0.4]"

position["random.png"] = "0 0.0"
position_x["random.png"] = 0
position_y["random.png"] = 0
position_x_offset["random.png"] = 0
position_y_offset["random.png"] = 0
size["random.png"] = "[220.1 231.5 0.4]"

position["simon_frasier_university.png"] = "0 0.0"
position_x["simon_frasier_university.png"] = -8
position_y["simon_frasier_university.png"] = 6
position_x_offset["simon_frasier_university.png"] = 0
position_y_offset["simon_frasier_university.png"] = 0
size["simon_frasier_university.png"] = "[195.12 100 0.4]"

position["klagenfurt_3.pgm"] = "0 0.0"
position_x["klagenfurt_3.pgm"] = -8
position_y["klagenfurt_3.pgm"] = 6
position_x_offset["klagenfurt_3.pgm"] = 0
position_y_offset["klagenfurt_3.pgm"] = 0
size["klagenfurt_3.pgm"] = "[125.6 120 0.4]"

position["klagenfurt_1.png"] = "0 0.0"
position_x["klagenfurt_1.png"] = -24
position_y["klagenfurt_1.png"] = 20
position_x_offset["klagenfurt_1.png"] = 0
position_y_offset["klagenfurt_1.png"] = 0
size["klagenfurt_1.png"] = "[293.45 250 0.4]"

position["empty.png"] = "0 0.0"
position_x["empty.png"] = 0
position_y["empty.png"] = 0
position_x_offset["empty.png"] = 0
position_y_offset["empty.png"] = 0
size["empty.png"] = "[400 400 0.4]"

position["small_map.png"] = "0 0.0"
position_x["small_map.png"] = -2
position_y["small_map.png"] = 3.5
position_x_offset["small_map.png"] = 1
position_y_offset["small_map.png"] = 0
size["small_map.png"] = "[10 10 0.4]"

position["office_1.png"] = "0 0.0"
position_x["office_1.png"] = 5.2
position_y["office_1.png"] = 0
position_x_offset["office_1.png"] = -1
position_y_offset["office_1.png"] = 0
size["office_1.png"] = "[20.0 22.3 0.5]"

position["office_2.png"]  = "0 180.0"
position_x["office_2.png"] = -10.5
position_y["office_2.png"] = 0.5
position_x_offset["office_2.png"] = 1
position_y_offset["office_2.png"] = 0
size["office_2.png"] = "[27.0 29.3 0.5]"

position["office_3.png"]  = "0 180.0"
position_x["office_3.png"] = -8.5
position_y["office_3.png"] = -3.5
position_x_offset["office_3.png"] = 1
position_y_offset["office_3.png"] = 0
size["office_3.png"] = "[20.0 9.0 0.5]"

position["office_4.png"] = "0 0"
position_x["office_4.png"] = -12
position_y["office_4.png"] = 3.3
position_x_offset["office_4.png"] = 0
position_y_offset["office_4.png"] = -2
offset_direction = "y"
size["office_4.png"] = "[26.0 14.8 0.5]"


position["warehouse.png"] = "0 90.0"
position_x["warehouse.png"] = 0.0
position_x["warehouse.png"] = 5.0
position_x_offset["warehouse.png"] = 1
position_x_offset["warehouse.png"] = 0
size["warehouse.png"] = "[28.0 20.03 0.5]"

position["awesome.png"] = "0 90.0"
position_x["awesome.png"] = 0
position_y["awesome.png"] = 4.0
position_x_offset["awesome.png"] = 1
position_y_offset["awesome.png"] = 0
size["awesome.png"] = "[14.0 10.03 0.5]"

position["maze_1.png"] = "0 0.0"
position_x["maze_1.png"] = 0
position_y["maze_1.png"] = 8.5
position_x_offset["maze_1.png"] = 1
position_y_offset["maze_1.png"] = 0
size["maze_1.png"] = "[21.0 19.03 0.5]"

position["hospital_full.png"] = "0 0.0"
position_x["hospital_full.png"] = -6.5
position_y["hospital_full.png"] = -35.3
position_x_offset["hospital_full.png"] = 1
position_y_offset["hospital_full.png"] = 0
size["hospital_full.png"] = "[181.0 79.03 0.5]"

position["hospital_short.png"] = "0 0.0"
position_x["hospital_short.png"] = -6.5
position_y["hospital_short.png"] = -35.3
position_x_offset["hospital_short.png"] = 1
position_y_offset["hospital_short.png"] = 0
size["hospital_short.png"] = "[181.0 79.03 0.5]"

position["empty_corridor.png"] = "0 0.180"
position_x["empty_corridor.png"] = -12
position_y["empty_corridor.png"] = 0.3
position_x_offset["empty_corridor.png"] = 0
position_y_offset["empty_corridor.png"] = -2
size["empty_corridor.png"] = "[26 1.7 0.5]"

position["open_space.png"] = "0 0.0"
position_x["open_space.png"] = -12
position_y["open_space.png"] = 3.3
position_x_offset["open_space.png"] = 0
position_y_offset["open_space.png"] = -2
size["open_space.png"] = "[26.0 15.0 0.5]"

position["rooms_empty.png"] = "0 0.0"
position_x["rooms_empty.png"] = -12
position_y["rooms_empty.png"] = 3.3
position_x_offset["rooms_empty.png"] = 0
position_y_offset["rooms_empty.png"] = -2
size["rooms_empty.png"] = "[26.0 15.0 0.5]"

position["rooms_obstacles.png"] = "0 0.0"
position_x["rooms_obstacles.png"] = -12
position_y["rooms_obstacles.png"] = 3.3
position_x_offset["rooms_obstacles.png"] = 0
position_y_offset["rooms_obstacles.png"] = -2
size["rooms_obstacles.png"] = "[26.0 15.0 0.5]"

position["rooms_more_obstacles.png"] = "0 0.0"
position_x["rooms_more_obstacles.png"] = -12
position_y["rooms_more_obstacles.png"] = 3.3
position_x_offset["rooms_more_obstacles.png"] = 0
position_y_offset["rooms_more_obstacles.png"] = -2
size["rooms_more_obstacles.png"] = "[26.0 15.0 0.5]"

position["rooms_exponential.png"] = "0 0.0"
position_x["rooms_exponential.png"] = -12
position_y["rooms_exponential.png"] = 3.3
position_x_offset["rooms_exponential.png"] = 0
position_y_offset["rooms_exponential.png"] = -2
size["rooms_exponential.png"] = "[26.0 15.0 0.5]"

if args.map.startswith("rooms_"):
    position[args.map] = "0 0 0"
    position_x[args.map] = -12
    position_y[args.map] = 3.3
    position_x_offset[args.map] = 0
    position_y_offset[args.map] = -2
    size[args.map] = "[26.0 15.0 0.5]"


if not map_file in position.keys():
    print "ERROR: do not have starting positions for robots for this map file. Aborting."
    exit(1)
if not map_file in size.keys():
    print "ERROR: do not have size information for this map file. Aborting."
    exit(1)











###############################################################################
# WRITING WORLD FILE
###############################################################################

map_file = args.map
world_dir = "world"
world_file = "world.world"
world_path = os.path.join(package_path,world_dir,world_file)

fh_world_file=open(world_path,"w")
fh_world_file.truncate()
fh_world_file.write("define block model\n(\n  size [0.5 0.5 0.5] \n  gui_nose 0\n)\n\n")

fh_world_file.write("define topurg ranger\n(\n	sensor(	\n	range [ 0.0  30.0 ]\n    fov 270.25\n	samples 1081\n  )\n\n    # generic model properties\n  color \"black\" \n  size [ 0.05 0.05 0.1 ]\n)\n\n")

fh_world_file.write("define erratic position\n(\n  #size [0.1 0.1 0.1]\n  size [0.2 0.2 0.7]\n  origin [-0.05 0 0 0]\n  gui_nose 1\n  drive \"diff\"\n  topurg(pose [ 0.050 0.000 -0.4 0.000 ])\n)\n\n")

fh_world_file.write("define floorplan model\n(\n  # sombre, sensible, artistic\n  color \"gray30\"\n\n  # most maps will need a bounding box\n  boundary 1\n\n  gui_nose 0\n  gui_grid 0\n   gui_move 0\n  gui_outline 0\n  gripper_return 0\n  fiducial_return 0\n  laser_return 1\n)\n\n# set the resolution of the underlying raytrace model in meters\n")

fh_world_file.write("resolution 0.02\n\ninterval_sim 100  # simulation timestep in milliseconds\n\n\nwindow\n(\n  size [ 745.000 448.000 ] \n  rotate [ 0.000 0.000 ]\n  scale 28.806 \n)\n")

fh_world_file.write("floorplan\n(\n  name \"%s\"\n  bitmap \"%s\"\n  size %s\n  pose [ 0 0.000 0 0.000 ]\n)\n" %(map_file,map_file,size[map_file]))
print ("Finished world header")

for num_robot in range(args.num_robots):
    x = position_x[map_file] + num_robot * position_x_offset[map_file]
    y = position_y[map_file] + num_robot * position_y_offset[map_file]
    fh_world_file.write('erratic( pose [ %f %f %s ] name "robot_%i" color "%s" )\n' % (x,
										       y,
                                                                                       position[map_file],
                                                                                       num_robot,
			                                                               colors[num_robot % len(colors)]))
fh_world_file.close()



















###############################################################################
# WRITING LAUNCH FILE
###############################################################################



def explorer():
    fh_launch_file.write("\t\t<include file=\"$(find explorer)/launch/exploration.launch\">\n")
    fh_launch_file.write("\t\t\t<arg name=\"robot_name\" value=\"%s\" />\n" %robot_name)
    fh_launch_file.write("\t\t\t<arg name=\"robot_prefix\" value=\"%s\" />\n" % robot_name_pref)
    fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
    fh_launch_file.write("\t\t\t<arg name=\"frontier_selection\" value=\"%s\" />\n" %args.exploration_strategy)
    fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"true\" />\n")
    fh_launch_file.write("\t\t</include>\n")



def adhoc_communication():
    fh_launch_file.write("\t\t<include file=\"$(find adhoc_communication)/launch/adhoc_communication.launch\">\n")
    fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
    if simulation:
        fh_launch_file.write("\t\t\t<arg name=\"robot_name\" value=\"%s\" />\n" %robot_name)
        fh_launch_file.write("\t\t\t<arg name=\"robot_prefix\" value=\"%s\" />\n" % robot_name_pref)
        fh_launch_file.write("\t\t\t<arg name=\"interface\" value=\"lo\" />\n")
        fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"true\" />\n")
        fh_launch_file.write("\t\t\t<arg name=\"mac\" value=\"%s\" />\n" %robot_macs[robot_name])
    else:
        fh_launch_file.write("\t\t\t<arg name=\"interface\" value=\"%s\" />\n" %args.interface)
    if simulation or args.connectivity:
        fh_launch_file.write("\t\t\t<arg name=\"sim_robot_macs\" value=\"%s\" />\n" %connected_macs[robot_name])
    if args.profile:
        fh_launch_file.write("\t\t\t<arg name=\"launch_prefix\" value=\"%s\" />\n" %args.profile)
    fh_launch_file.write("\t\t</include>\n")



def emulate_comm():
    fh_launch_file.write("\t<include file=\"$(find adhoc_communication)/launch/adhoc_communication.launch\">\n")
    fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"true\" />\n")
    fh_launch_file.write("\t\t\t<arg name=\"simulation_mode\" value=\"true\" />\n")
    fh_launch_file.write("\t\t<arg name=\"num_of_robots\" value=\"%s\" />\n" % args.num_robots)
    #fh_launch_file.write("\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
    fh_launch_file.write("\t\t<arg name=\"emulate\" value=\"true\" />\n")
    fh_launch_file.write("\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    fh_launch_file.write("\t</include>\n")

def traffic_generator():
    fh_launch_file.write("\t\t<include file=\"$(find traffic_generator)/launch/traffic_generator.launch\">\n")
    if simulation:
        fh_launch_file.write("\t\t\t<arg name=\"robot_name\" value=\"%s\" />\n" %robot_name)
        fh_launch_file.write("\t\t\t<arg name=\"robot_prefix\" value=\"%s\" />\n" %robot_name_pref)
        fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"true\" />\n")
    fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
    fh_launch_file.write("\t\t\t<arg name=\"config_file\" value=\"%s\" />\n" %args.traffic_generator)
    fh_launch_file.write("\t\t</include>\n")

def traffic_receiver():
    fh_launch_file.write("\t\t<include file=\"$(find traffic_generator)/launch/traffic_receiver_standalone.launch\">\n")
    if simulation:
        fh_launch_file.write("\t\t\t<arg name=\"robot_name\" value=\"%s\" />\n" %robot_name)
        fh_launch_file.write("\t\t\t<arg name=\"robot_prefix\" value=\"%s\" />\n" %robot_name_pref)
        fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"true\" />\n")
    fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
    fh_launch_file.write("\t\t</include>\n")


def connection_manager():
    fh_launch_file.write("\t\t<include file=\"$(find connection_manager)/launch/connection_manager.launch\">\n")
    fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
    fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    if simulation:
        fh_launch_file.write("\t\t\t<arg name=\"robot_name\" value=\"%s\" />\n" %robot_name)
        fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"true\" />\n")
        fh_launch_file.write("\t\t\t<arg name=\"robot_prefix\" value=\"%s\" />\n" % robot_name_pref)
    fh_launch_file.write("\t\t</include>\n")


def mapping():
    fh_launch_file.write("\t\t<include file=\"$(find multi_robot_simulation)/launch/mapping.launch\">\n")
    fh_launch_file.write("\t\t\t<arg name=\"robot\" value=\"%s\" />\n" %robot_name)
    fh_launch_file.write("\t\t\t<arg name=\"robot_prefix\" value=\"%s\" />\n" %robot_name_pref)
    fh_launch_file.write("\t\t\t<arg name=\"robot_local_map_frame\" value=\"%s\" />\n" %robot_local_map_frame )
    fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
    fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"true\" />\n")
    fh_launch_file.write("\t\t\t<arg name=\"xmax\" value=\"%d\" />\n" %args.map_size[0])
    fh_launch_file.write("\t\t\t<arg name=\"xmin\" value=\"-%d\" />\n" %args.map_size[0])
    fh_launch_file.write("\t\t\t<arg name=\"ymax\" value=\"%d\" />\n" %args.map_size [1])
    fh_launch_file.write("\t\t\t<arg name=\"ymin\" value=\"-%d\" />\n" %args.map_size[1])
    fh_launch_file.write("\t\t</include>\n")



def map_merger():
    fh_launch_file.write("\t\t<include file=\"$(find map_merger)/launch/map_merger.launch\">\n")
    fh_launch_file.write("\t\t\t<arg name=\"robot_name\" value=\"%s\" />\n"% robot_name)
    fh_launch_file.write("\t\t\t<arg name=\"robot_prefix\" value=\"%s\" />\n"%robot_name_pref)
    fh_launch_file.write("\t\t\t<arg name=\"robot_local_map_frame\" value=\"%s\" />\n"%robot_local_map_frame )
    fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
    fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"true\" />\n")
    fh_launch_file.write("\t\t</include>\n")


def move_base():
    fh_launch_file.write("\t\t<include file=\"$(find multi_robot_simulation)/launch/move_base.launch\">\n")
    fh_launch_file.write("\t\t\t<arg name=\"robot\" value=\"%s\" />\n" %robot_name)
    fh_launch_file.write("\t\t\t<arg name=\"robot_pref\" value=\"%s\" />\n" %robot_name_pref)
    fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    fh_launch_file.write("\t\t</include>\n")


def fake_mapping():
    origin = size[map_file].lstrip("[]").split()
    fh_launch_file.write("\t\t<include file=\"$(find fake_mapping)/launch/fake_mapping.launch\">\n")
    fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
    fh_launch_file.write("\t\t\t<arg name=\"robot_name\" value=\"%s\" />\n" %robot_name)
    fh_launch_file.write("\t\t\t<arg name=\"image\" value=\"$(find multi_robot_simulation)/world/%s\" />\n" %map_file)
    fh_launch_file.write("\t\t\t<arg name=\"origin_x\" value=\"%f\" />\n" %(-(float(origin[0])/2.0 + float(position_x[map_file]))))
    fh_launch_file.write("\t\t\t<arg name=\"origin_y\" value=\"%f\" />\n" %(-(float(origin[1])/2.0 + float(position_y[map_file]))))
    fh_launch_file.write("\t\t</include>\n")


def fake_localization():
    fh_launch_file.write("\t\t<include file=\"$(find multi_robot_simulation)/launch/fake_localization.launch\">\n")
    fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
    fh_launch_file.write("\t\t\t<arg name=\"robot_prefix\" value=\"%s\" />\n"%robot_name_pref)
    fh_launch_file.write("\t\t\t<arg name=\"position_x\" value=\"%f\" />\n"%position_x[map_file])
    fh_launch_file.write("\t\t\t<arg name=\"position_y\" value=\"%f\" />\n"%position_y[map_file])
    fh_launch_file.write("\t\t</include>\n")



fh_launch_file = open(launch_path,"w")
fh_launch_file.truncate()
print ("Writing File headers")
fh_launch_file.write("<?xml version=\"1.0\"?>\n")
fh_launch_file.write("<launch>\n")

robot_name_pref = ""

if simulation:
    if args.gui:
        # disable GUI by adding flag -g to stageros
        fh_launch_file.write("\t<node name=\"stage\" pkg =\"stage_ros\" type=\"stageros\" args=\"$(find multi_robot_simulation)/world/world.world\" />\n")
    else:
        fh_launch_file.write("\t<node name=\"stage\" pkg =\"stage_ros\" type=\"stageros\" args=\"-g $(find multi_robot_simulation)/world/world.world\" />\n")
if simulation:
    fh_launch_file.write("\t<param name=\"use_sim_time\"  value=\"true\"/>\n")

for num_robot in range(args.num_robots):
    if args.robot_id:
        num_robot = args.robot_id
    if simulation:
        robot_name = "robot_" + str(num_robot)
        if args.num_robots > 1:
            robot_name_pref ="/robot_" + str(num_robot)
    else:
        robot_name = socket.gethostname()
    # start connection manager in any case
    if args.num_robots > 1:
        fh_launch_file.write("\t<group ns=\"robot_%i\">\n" %num_robot)
	robot_local_map_frame=robot_name + "/map"
    else:
	robot_local_map_frame="map"
    connection_manager()



    # adhoc_communication node
    if not args.emulate_comm:
        adhoc_communication()

    if args.simulate_exploration:
        move_base()
        #mapping()
        fake_mapping()
        fake_localization()
        map_merger()
        explorer()
    elif args.test_communication:
        traffic_receiver()



    if args.traffic_generator:
        traffic_generator()
    elif args.traffic_receiver:
        traffic_receiver()
    if args.num_robots > 1:
        fh_launch_file.write("\t</group>\n")
    if args.robot_id != None:
        # write only a single config file in case the robot id is set
        break

# we emulate so the adhoc communication node does not need to be
# started for every robot
if args.emulate_comm:
    emulate_comm()




fh_launch_file.write("</launch>\n")
fh_launch_file.close()


















###############################################################################
# WRITING META FILE (DONE IN EVERY CASE)
###############################################################################

os.makedirs(log_path)
meta_file = os.path.join(log_path,"..","meta.log")
print "Creating file %s" %meta_file
if not os.path.isfile(meta_file):
    open(meta_file,"w").close()
fh_meta_log = open(meta_file, "r")
config = ConfigParser.ConfigParser()
config.read(meta_file)
fh_meta_log.close()
if not config.has_section("meta"):
    config.add_section("meta")
    config.set("meta", "map", "%s" %map_file)
    config.set("meta", "created_on", "%s" %time.ctime())
    config.set("meta", "num_robots", "%d" %args.num_robots)
    config.set("meta", "host", "%s" %socket.gethostname())
if not config.has_section("Runs"):
    config.add_section("Runs")
config.set("Runs", "start_time_run_%s" %args.sim_id, "%s" %time.ctime())
fh_meta_log = open(meta_file, "w")
config.write(fh_meta_log)
fh_meta_log.close()






os.system("roslaunch multi_robot_simulation %s" %launch_file)
