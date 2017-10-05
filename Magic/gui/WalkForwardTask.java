package Magic.gui;

import nl.dke.pursuitevasion.game.Vector2D;

/**
 * Created by Jan on 16-5-2017.
 */
public class WalkForwardTask extends AbstractAgentTask {

    double walkScale;

    public WalkForwardTask(double scale){
        super();
        walkScale = scale;
    }

    @Override
    protected AgentCommand computeAgentCommand(AbstractAgent agent, double maxDistance, double maxRotation) {
        double angle = agent.getFacingAngle();
        angle = Math.toRadians(angle);
        double opposite = -Math.sin(angle)* maxDistance * walkScale;
        double adjacent = Math.cos(angle) * maxDistance* walkScale;

        Vector2D location = agent.getLocation();
        Vector2D newLocation = location.add(adjacent, opposite);

        return new AgentCommand(agent, newLocation);
    }

    @Override
    protected boolean completesTask(AgentCommand command) {
        return true;
    }
}
