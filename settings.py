import os, random

# Directories
MAIN_DIR       = os.path.dirname(os.path.realpath(__file__))
SCENES         = MAIN_DIR + '/scenes'
VREP_DIR       = MAIN_DIR + '/vrep'

sim_port       = 19999
local_ip       = '127.0.0.1'


# Simulation parameters
fps = 30.               # Frames per second
simulation_steps = 5000
image_size = 800
map_size   = 80
steps_slam = 1 # Perform SLAM every n steps
steps_lee = 10 # Perform path planning every n steps
cluster_threshold = 5 # threshold t for scipy.cluster.hierarchy.fclusterdata
cluster_filter = 5 # clusters with pixels less than this number will be deleted
