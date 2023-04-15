package org.sciborgs1155.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.subsystems.LED;
import org.sciborgs1155.robot.util.placement.PlacementState;
import org.sciborgs1155.robot.util.placement.PlacementState.GamePiece;
import org.sciborgs1155.robot.util.placement.PlacementState.Level;
import org.sciborgs1155.robot.util.placement.PlacementState.Side;

public final class Scoring implements Sendable {

  private final Placement placement;
  private final LED led;

  private Side side = Side.BACK;
  private GamePiece gamePiece = GamePiece.CONE;

  public Scoring(Placement placement, LED led) {
    this.placement = placement;
    this.led = led;
  }

  public Command setGamePiece(GamePiece gamePiece) {
    return Commands.runOnce(() -> this.gamePiece = gamePiece)
        .alongWith(led.setGamePieceColor(gamePiece));
  }

  public Command setSide(Side side) {
    return Commands.runOnce(() -> this.side = side);
  }

  /**
   * Returns an appropriate {@link PlacementState} based on current parameters and inputted {@link
   * PlacementState.Level}
   */
  public PlacementState state(Level level) {
    return PlacementState.fromOperator(level, gamePiece, side);
  }

  public GamePiece gamePiece() {
    return gamePiece;
  }

  public Side side() {
    return side;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Game Piece", () -> gamePiece.name(), null);
    builder.addStringProperty("Side", () -> side.name(), null);
  }
}
