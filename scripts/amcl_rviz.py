import os
import sys
from multiprocessing import Pool

processes = ["color_detection amcl.launch", "turtlebot_rviz_launchers view_navigation.launch --screen"]
def run_processes(process):
    os.system('roslaunch {}'.format(process))

if __name__ == "__main__":

    if len(sys.argv) > 1:
        arg1 = sys.argv[1]
        if arg1 == "physical":
            processes[0] = "color_detection amcl_zones.launch"
        elif arg1 == "sketched":
            processes[0] = "color_detection amcl_sketched_zones.launch"
        elif arg1 == "virtual":
            processes[0] = "color_detection amcl_virtual_zones.launch"
        else: # directory of map file
            processes[0] += " map_file:={}".format(arg1)

        # feedback type and map file given
        if len(sys.argv) == 3:
            arg2 = sys.argv[2]
            processes[0] += " map_file:={}".format(arg2)


    pool = Pool(processes=2)
    pool.map(run_processes, processes)
