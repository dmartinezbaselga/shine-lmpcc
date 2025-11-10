# evaluation_scripts
recording_to_samples.py: Is the script that takes in a .bag file, extracts the requried measurements and partitions them.
partition.py: is a library to observation_partitioning_node.py, which helps in converting all raw sensor readings to processed form (For example getting distance to sidewalk/crosswalk and localised velocities).
main.py (name will be changed) - Is a script that takes in the processed sensor readings, and assigns a partition to each pedestrian in each scene.
trial.py (name will be changed) - Is a utility script ot store each of the partitions in the form of a json file.

Files to check the recall and variance offline
json_read.cpp: Is a cpp file to read each test case and collect and assign the required scenarios from the database and thus store the predictions for each test case.
plot_trajectory.py: Is a file to plot each prediction, and calculate the recall and variance for the same.


Files used in scrips folder of smpcc to partition each pedestrian online:
observation_partitioning_node.py (Also present in the smpcc repo)
