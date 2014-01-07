package champ2010client;

/**
 * Created by IntelliJ IDEA.
 * User: jtogel
 * Date: 12-Oct-2006
 * Time: 19:06:37
 */
public abstract class CMAEvolvable extends Controller {

    public abstract double[] getParameters();
    
    public abstract void setParameters(double[] parameters);

    public abstract double[] getInitialDeviationParameters();
    
    public abstract boolean isFeasible();
}
