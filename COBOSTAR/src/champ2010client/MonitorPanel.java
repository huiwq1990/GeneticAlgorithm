package champ2010client;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Polygon;
import java.util.Vector;

import javax.swing.JPanel;

class MonitorPanel extends JPanel
{
	private static final long serialVersionUID = 1L;

	private int carWidth = 2;
	private int carLength = 5;
	private double zoomFactor = 1.5;//ML
	private int panelWidth = 450;
	private int panelHeight = 450;
	private int opponentRadius = 5;
	private double old = 0;
	double[] focusSensors = new double[5];

	private Polygon trackBorders;
	private Polygon trackFocus;//ML
	private Vector<Car> opponentData;

	public MonitorPanel()
	{
		opponentData = new Vector<Car>();
	}

	public Dimension getPreferredSize()
	{
		return new Dimension(panelWidth, panelHeight);
	}

	public void paintComponent(Graphics g)
	{
		super.paintComponent(g);

		g.setColor(Color.GRAY);
		g.drawOval((int)(panelWidth - 200*zoomFactor)/2, (int)(panelHeight - 100*zoomFactor)/2, (int)(200*zoomFactor), (int)(200*zoomFactor));
		g.setColor(Color.LIGHT_GRAY);
		g.drawOval((int)(panelWidth - 400*zoomFactor)/2, (int)(panelHeight - 300*zoomFactor)/2, (int)(400*zoomFactor), (int)(400*zoomFactor));//ML

		g.translate((panelWidth/2), (int)(((panelHeight-100*zoomFactor)/2)+100*zoomFactor));
		g.setColor(Color.GRAY);
		g.drawPolygon(trackBorders);

		if(old < 0.5)	g.setColor(Color.ORANGE);//ML Highlight polygon when currently refreshed
		else			g.setColor(Color.YELLOW);//ML
		if(old > 0)
			g.drawPolygon(trackFocus);//ML

		g.setColor(Color.BLACK);
		g.drawRect((int)(-(carWidth/2)*zoomFactor), (int)(-(carLength/2)*zoomFactor), (int)(carWidth * zoomFactor), (int)(carLength * zoomFactor));

		for (Car opponent : opponentData)
		{
			double angle = ((opponent.getSector() * 10) + 5) * Math.PI / 180;
			double x = Math.sin(angle) * opponent.getDistance();
			double y = Math.cos(angle) * opponent.getDistance();

			g.setColor(Color.GRAY);
			g.drawOval((int)((-(int) Math.round(x + Math.sin(Math.PI- opponent.getDirection()) * opponent.getVelocity()) - opponentRadius/2) * zoomFactor),
					(int)(((int) Math.round(y + Math.cos(Math.PI- opponent.getDirection()) * opponent.getVelocity()) - opponentRadius/2) * zoomFactor),
					(int)(opponentRadius*zoomFactor), (int)(opponentRadius*zoomFactor));

			if (opponent.isVisible())
			{
				g.setColor(Color.RED);
			}
			else
			{
				g.setColor(Color.YELLOW);
			}
			g.drawOval((int)((-(int) Math.round(x) - opponentRadius/2) * zoomFactor), (int)(((int) Math.round(y) - opponentRadius/2) * zoomFactor), (int)(opponentRadius*zoomFactor), (int)(opponentRadius*zoomFactor));
			g.setColor(Color.BLACK);
			g.drawString(opponentData.indexOf(opponent) + "", (int)((-(int) Math.round(x) - opponentRadius/2) * zoomFactor), (int)(((int) Math.round(y) - opponentRadius/2) * zoomFactor));
		}
	}

	public void visualizeSensors(double[] trackEdgeSensors, Vector<Car> oD, int _focusAngle, double[] fS)
	{
		trackBorders = new Polygon();
		focusSensors = fS;
		
		for (int i=0; i<=18; i++)
		{
			double angle = i * 10 * Math.PI / 180;
			double x = Math.cos(angle) * trackEdgeSensors[i];
			double y = Math.sin(angle) * trackEdgeSensors[i];
			trackBorders.addPoint((int)((-1.0)*x*zoomFactor), (int)((-1.0)*y*zoomFactor));
		}
		//ML################## Focus Sensors ##################
		double angle1 = Math.PI / 180;
		double angle2 = 180 * Math.PI / 180;
		double x1 = Math.cos(angle1) * 0.1;
		double x2 = Math.cos(angle2) * 0.1;
		double y1 = Math.sin(angle1) * 0.1;
		double y2 = Math.sin(angle2) * 0.1;
		if(focusSensors[0] != -1) {
			old = 0;
			trackFocus = new Polygon();//ML
			trackFocus.addPoint((int)(-(int) Math.round(x1)*zoomFactor), (int)(-(int) Math.round(y1)*zoomFactor));
			trackFocus.addPoint((int)(-(int) Math.round(x2)*zoomFactor), (int)(-(int) Math.round(y2)*zoomFactor));
		}else{
			old += 0.1;
		}
		
		double angle[] = new double[5]; 
		angle[0] = (_focusAngle+88) * Math.PI / 180;
		angle[1] = (_focusAngle+89) * Math.PI / 180;
		angle[2] = (_focusAngle+90) * Math.PI / 180;
		angle[3] = (_focusAngle+91) * Math.PI / 180;
		angle[4] = (_focusAngle+92) * Math.PI / 180;

		for (int i=0; i<=4; i++)
		{
			if(focusSensors[i] != -1)
			{
				double x = Math.cos(angle[i]) * focusSensors[i];
				double y = Math.sin(angle[i]) * focusSensors[i];
				trackFocus.addPoint((int)(-(int) Math.round(x)*zoomFactor), (int)(-(int) Math.round(y)*zoomFactor));
			}
		}
		//######################################################

		opponentData = oD;

		repaint();
	}
}