package org.sciborgs1155.robot.subsystems.placement;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;

public class SolverClient {

    private final StringPublisher requester;
    private final DoubleArraySubscriber results;

    public SolverClient() {
        var chronosTable = NetworkTableInstance.getDefault().getTable("chronos");
        requester = chronosTable.getStringTopic("request").publish(PubSubOption.periodic(0.0), PubSubOption.keepDuplicates(true));
        results = chronosTable.getDoubleArrayTopic("result").subscribe(new double[] {}, PubSubOption.periodic(0.0));
    }

    public void request(State initialState, State finalState) {
        
    }



}
