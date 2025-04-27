import argparse
import logging
import sys

sys.path.append("..")
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import rtde.csv_writer as csv_writer
import rtde.csv_binary_writer as csv_binary_writer

conf = rtde_config.ConfigFile("record_configuration.xml")
output_names, output_types = conf.get_recipe("out")

con = rtde.RTDE("192.168.3.101", 30004)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
if not con.send_output_setup(output_names, output_types, frequency=500):
    logging.error("Unable to configure output")
    sys.exit()

# start data synchronization
if not con.send_start():
    logging.error("Unable to start synchronization")
    sys.exit()

with open("robot_data.csv", "w") as csvfile:
    writer = csv_writer.CSVWriter(csvfile, output_names, output_types)

    writer.writeheader()

    i = 1
    keep_running = True
    while keep_running:

        if i % 500 == 0:
            sys.stdout.write("\r")
            sys.stdout.write("{:3d} samples.".format(i))
            sys.stdout.flush()
        try:
            # state = con.receive_buffered(args.binary)
            state = con.receive(False)
            if state is not None:
                writer.writerow(state)
                i += 1

        except KeyboardInterrupt:
            keep_running = False
        except rtde.RTDEException:
            con.disconnect()
            sys.exit()


sys.stdout.write("\rComplete!            \n")

con.send_pause()
con.disconnect()
