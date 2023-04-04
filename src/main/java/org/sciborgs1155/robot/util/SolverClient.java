package org.sciborgs1155.robot.util;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonGenerator;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import java.io.ByteArrayOutputStream;
import java.io.IOException;

public class SolverClient {

  private final StringPublisher requestPub;
  private final DoubleArraySubscriber resultPub;

  public SolverClient() {
    var chronosTable = NetworkTableInstance.getDefault().getTable("chronos");
    requestPub =
        chronosTable
            .getStringTopic("request")
            .publish(PubSubOption.periodic(0.0), PubSubOption.keepDuplicates(true));
    resultPub =
        chronosTable
            .getDoubleArrayTopic("result")
            .subscribe(new double[] {}, PubSubOption.periodic(0.0));
  }

  public void request(Vector<N3> initialState, Vector<N3> finalState) {
    ByteArrayOutputStream stream = new ByteArrayOutputStream();
    JsonGenerator generator;
    try {
      generator = new JsonFactory().createGenerator(stream);
      generator.writeStartObject();

      generator.writeArrayFieldStart("initial");
      generator.writeNumber(initialState.get(0, 0));
      generator.writeNumber(initialState.get(1, 0));
      generator.writeNumber(initialState.get(2, 0));
      generator.writeEndArray();

      generator.writeArrayFieldStart("final");
      generator.writeNumber(finalState.get(0, 0));
      generator.writeNumber(finalState.get(1, 0));
      generator.writeNumber(finalState.get(2, 0));
      generator.writeEndArray();

      // TODO: ADD CONSTRAINT PARAMETERS

      generator.writeEndObject();
      generator.close();
      stream.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    requestPub.set(stream.toString());
  }

  // public void results(Trajectory trajectory) {}
}
