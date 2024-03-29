# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
# Copyright (c) 2023 FRC 1155
# https://github.com/SciBorgs

import json
import time

import ntcore

from solver import Solver

SERVER_IP = "127.0.0.1"

solver = None
result_pub = None


def config_update(event):
    global solver
    print("Creating new solver...")
    config = json.loads(event.data.value.getString())
    solver = Solver(config)
    print("Solver ready")


def request_update(event):
    global solver, result_pub
    if solver == None:
        print("Skipping request, solver not available")
    print("Solving trajectory...")
    parameters = json.loads(event.data.value.getString())
    start_time = time.time()
    result = solver.solve(parameters)
    end_time = time.time()
    print()
    if result != None:
        print("Trajectory generated")
        print("\tGenTime = " + str(end_time - start_time))
        print("\tPathTime = " + str(result[0]))
        print("\tDT = " + str(result[0] / (len(result[1]) - 1)))
        points = [parameters["hash"], result[0]]
        for i in range(len(result[1])):
            points.append(result[1][i])
            points.append(result[2][i])
            points.append(result[3][i])
        result_pub.set(points)
    else:
        print("Trajectory generation failed")


if __name__ == "__main__":
    # Set up NT instance
    nt_inst = ntcore.NetworkTableInstance.getDefault()
    nt_inst.setServer(SERVER_IP)
    nt_inst.startClient4("chronos")

    # Create subscribers and publisher
    config_sub = nt_inst.getStringTopic("/chronos/config").subscribe(
        "", ntcore.PubSubOptions(periodic=0)
    )
    request_sub = nt_inst.getStringTopic("/chronos/request").subscribe(
        "", ntcore.PubSubOptions(periodic=0)
    )
    result_pub = nt_inst.getDoubleArrayTopic(
        "/chronos/result"
    ).publish(ntcore.PubSubOptions(periodic=0))
    ping_pub = nt_inst.getIntegerTopic("/chronos/ping").publish(
        ntcore.PubSubOptions()
    )

    # Create event listeners
    nt_inst.addListener(config_sub, ntcore.EventFlags.kValueRemote, config_update)
    nt_inst.addListener(request_sub, ntcore.EventFlags.kValueRemote, request_update)

    # Run forever
    i = 0
    while True:
        time.sleep(0.1)
        i = (i + 1) % 10
        ping_pub.set(i)