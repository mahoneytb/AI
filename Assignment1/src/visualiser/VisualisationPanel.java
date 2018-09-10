package visualiser;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;

import javax.sound.sampled.Line;
import javax.swing.JComponent;
import javax.swing.Timer;
import javax.swing.plaf.nimbus.State;

import org.w3c.dom.css.Rect;
import problem.Box;
import problem.MovingBox;
import problem.MovingObstacle;
import problem.StaticObstacle;
import problem.ProblemSpec;
import problem.RobotConfig;

public class VisualisationPanel extends JComponent {
    /** UID, as required by Swing */
    private static final long serialVersionUID = -4286532773714402501L;

    private ProblemSpec problemSetup = new ProblemSpec();
    private Visualiser visualiser;

    private AffineTransform translation = AffineTransform.getTranslateInstance(
            0, -1);
    private AffineTransform transform = null;

    // State Information
    private RobotConfig currentRobotConfig;
    private List<Box> currentMovingBoxes;
    private List<Box> currentMovingObstacles;

    private boolean animating = false;
    private boolean displayingSolution = false;
    private Timer animationTimer;
    private int framePeriod = 20; // 50 FPS
    private Integer frameNumber = null;
    private int maxFrameNumber;

    private int samplingPeriod = 100;

    public VisualisationPanel(Visualiser visualiser) {
        super();
        this.setBackground(Color.WHITE);
        this.setOpaque(true);
        this.visualiser = visualiser;
    }

    public void setDisplayingSolution(boolean displayingSolution) {
        this.displayingSolution = displayingSolution;
        repaint();
    }

    public boolean isDisplayingSolution() {
        return displayingSolution;
    }

    public void setFramerate(int framerate) {
        this.framePeriod = 1000 / framerate;
        if (animationTimer != null) {
            animationTimer.setDelay(framePeriod);
        }
    }

    public void initAnimation() {
        if (!problemSetup.getSolutionLoaded()) {
            return;
        }
        if (animationTimer != null) {
            animationTimer.stop();
        }
        animating = true;
        gotoFrame(0);
        maxFrameNumber = problemSetup.getRobotPath().size() - 1;
        animationTimer = new Timer(framePeriod, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent arg0) {
                int newFrameNumber = frameNumber + 1;
                if (newFrameNumber >= maxFrameNumber) {
                    animationTimer.stop();
                    visualiser.setPlaying(false);
                }
                if (newFrameNumber <= maxFrameNumber) {
                    gotoFrame(newFrameNumber);
                }
            }
        });
        visualiser.setPlaying(false);
        visualiser.updateMaximum();
    }

    public void gotoFrame(int frameNumber) {
        if (!animating
                || (this.frameNumber != null && this.frameNumber == frameNumber)) {
            return;
        }
        this.frameNumber = frameNumber;
        visualiser.setFrameNumber(frameNumber);
        currentRobotConfig = problemSetup.getRobotPath().get(frameNumber);
        currentMovingBoxes = problemSetup.getMovingBoxPath().get(frameNumber);
        currentMovingObstacles = problemSetup.getMovingObstaclePath().get(frameNumber);
        repaint();
    }

    public int getFrameNumber() {
        return frameNumber;
    }

    public void playPauseAnimation() {
        if (animationTimer.isRunning()) {
            animationTimer.stop();
            visualiser.setPlaying(false);
        } else {
            if (frameNumber >= maxFrameNumber) {
                gotoFrame(0);
            }
            animationTimer.start();
            visualiser.setPlaying(true);
        }
    }

    public void stopAnimation() {
        if (animationTimer != null) {
            animationTimer.stop();
        }
        animating = false;
        visualiser.setPlaying(false);
        frameNumber = null;
    }

    public ProblemSpec getProblemSetup() {
        return problemSetup;
    }

    public void calculateTransform() {
        transform = AffineTransform.getScaleInstance(getWidth(), -getHeight());
        transform.concatenate(translation);
    }

    public void paintState(Graphics2D g2, RobotConfig rc, List<Box> mb, List<Box> mo) {
        if ((rc == null)||(mb == null)||(mo == null)) {
            return;
        }
        // Define Robot
        Line2D.Float robot = new Line2D.Float(rc.getX1(problemSetup.getRobotWidth()),
                rc.getY1(problemSetup.getRobotWidth()),
                rc.getX2(problemSetup.getRobotWidth()),
                rc.getY2(problemSetup.getRobotWidth()));

        // Draw Moving Boxes
        g2.setColor(Color.blue);
        g2.draw(robot);
        for (Box box : mb) {
            g2.draw(transform.createTransformedShape(box.getRect()));
        }

        // Draw Moving Obstacles
        g2.setColor(Color.orange);
        g2.draw(robot);
        for (Box box : mo) {
            g2.draw(transform.createTransformedShape(box.getRect()));
        }

        // Draw Robot last so it's on top
        g2.setColor(Color.black);
        g2.setStroke(new BasicStroke(2));
        g2.draw(transform.createTransformedShape(robot));

    }

    public void setSamplingPeriod(int samplingPeriod) {
        this.samplingPeriod = samplingPeriod;
        repaint();
    }

    public void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        if (!problemSetup.getProblemLoaded()) {
            return;
        }
        calculateTransform();
        Graphics2D g2 = (Graphics2D) graphics;
        g2.setColor(Color.WHITE);
        g2.fillRect(0, 0, getWidth(), getHeight());

        List<StaticObstacle> obstacles = problemSetup.getStaticObstacles();
        if (obstacles != null) {
            g2.setColor(Color.red);
            for (StaticObstacle obs : obstacles) {
                Shape transformed = transform.createTransformedShape(obs
                        .getRect());
                g2.draw(transformed);
            }
        }

        List<Box> movingBoxEndPositions = generateMovingBoxes(problemSetup.getMovingBoxEndPositions());
        g2.setColor(Color.green);
        for (Box box : movingBoxEndPositions) {
            Shape transformed = transform.createTransformedShape(box
                    .getRect());
            g2.draw(transformed);
        }

        g2.setStroke(new BasicStroke(2));
        if (!animating) {
            if (displayingSolution) {
                List<RobotConfig> robotConfigPath = problemSetup.getRobotPath();
                List<List<Box>> movingBoxPath = problemSetup.getMovingBoxPath();
                List<List<Box>> movingObstaclePath = problemSetup.getMovingObstaclePath();
                int lastIndex = robotConfigPath.size() - 1;
                for (int i = 0; i < lastIndex; i += samplingPeriod) {
                    float t = (float) i / lastIndex;
                    g2.setColor(new Color(0, t, 1 - t));
                    paintState(g2, robotConfigPath.get(i), movingBoxPath.get(i), movingObstaclePath.get(i));
                }
                g2.setColor(Color.green);
                paintState(g2, robotConfigPath.get(lastIndex), movingBoxPath.get(lastIndex), movingObstaclePath.get(lastIndex));
            } else {
//                g2.setColor(Color.blue);
                paintState(g2, problemSetup.getInitialRobotConfig(),
                        problemSetup.getMovingBoxes(),
                        problemSetup.getMovingObstacles());
//                g2.setColor(Color.green);
//                paintState(g2, problemSetup.getInitialRobotConfig(),
//                        generateMovingBoxes(problemSetup.getMovingBoxEndPositions()),
//                        problemSetup.getMovingObstacles());
            }
        } else {
            g2.setColor(Color.blue);
            paintState(g2, currentRobotConfig, currentMovingBoxes, currentMovingObstacles);
        }
    }

    public List<Box> generateMovingBoxes(List<Point2D> movingBoxCentres) {
        List<Box> movingBoxes = new ArrayList<>();
        Point2D centre;
        for (int i = 0; i < movingBoxCentres.size(); i++) {
            centre = movingBoxCentres.get(i);
            movingBoxes.add(new MovingBox(
                    centre,	problemSetup.getRobotWidth()));
        }
        return movingBoxes;
    }


}
