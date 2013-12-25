package hg.pacman;

import static pacman.game.Constants.DELAY;

import java.util.EnumMap;
import java.util.Random;

import pacman.controllers.Controller;
import pacman.controllers.HumanController;
import pacman.game.Game;
import pacman.game.GameView;
import pacman.game.Constants.GHOST;
import pacman.game.Constants.MOVE;

public class PacmanEvaluate {

	
	 

	public static void runGameTimed(Controller<MOVE> pacManController,Controller<EnumMap<GHOST,MOVE>> ghostController,boolean visual)
		{
			Game game=new Game(0);
			
			GameView gv=null;
			
			if(visual)
				gv=new GameView(game).showGame();
			
			if(pacManController instanceof HumanController)
				gv.getFrame().addKeyListener(((HumanController)pacManController).getKeyboardInput());
					
			new Thread(pacManController).start();
			new Thread(ghostController).start();
			
			while(!game.gameOver())
			{
				pacManController.update(game.copy(),System.currentTimeMillis()+DELAY);
				ghostController.update(game.copy(),System.currentTimeMillis()+DELAY);

				try
				{
					Thread.sleep(DELAY);
				}
				catch(InterruptedException e)
				{
					e.printStackTrace();
				}

		        game.advanceGame(pacManController.getMove(),ghostController.getMove());	   
		        
		        if(visual)
		        	gv.repaint();
			}
			
			pacManController.terminate();
			ghostController.terminate();
		}
	
	public static double runExperiment(Controller<MOVE> pacManController,
			Controller<EnumMap<GHOST, MOVE>> ghostController, int trials) {
		double avgScore = 0;

		Random rnd = new Random(0);
		Game game;

		for (int i = 0; i < trials; i++) {
			game = new Game(rnd.nextLong());

			while (!game.gameOver()) {
				game.advanceGame(
						pacManController.getMove(game.copy(),
								System.currentTimeMillis() + DELAY),
						ghostController.getMove(game.copy(),
								System.currentTimeMillis() + DELAY));
			}

			avgScore += game.getScore();
//			System.out.println(i + "\t" + game.getScore());
		}

//		System.out.println(avgScore / trials);
		return avgScore / trials;
	}

}
