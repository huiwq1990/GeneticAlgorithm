/*
 * Copyright (C) 2004 Derek James and Philip Tucker
 * 
 * This file is part of ANJI (Another NEAT Java Implementation).
 * 
 * ANJI is free software; you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with this program; if
 * not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307 USA
 * 
 * Created on Apr 28, 2005 by Philip Tucker
 */
package com.anji.tournament;

/**
 * 
 * @author Philip Tucker
 */
public class PlayerResults {

private Player player;

private PlayerStats stats = new PlayerStats();

private ScoringWeights weights = null;

/**
 * ctor  
 * @param aPlayer
 * @param aWeights
 */
public PlayerResults( Player aPlayer, ScoringWeights aWeights ) {
	player = aPlayer;
	weights = aWeights;
}

/**
 * ctor
 * @param aPlayer
 */
public PlayerResults( Player aPlayer ) {
	this( aPlayer, null );
}

/**
 * @return subject
 */
public Player getPlayer() {
	return player;
}

/**
 * @return stats
 */
public PlayerStats getResults() {
	return stats;
}

/**
 * @return score
 */
public float getScore() {
	return ( weights == null ) ? 0 : weights.calculateTotalScore( stats );
}

/**
 * @see java.lang.Object#toString()
 */
public String toString() {
	return new StringBuffer().append( player.getPlayerId() ).append( ": " ).append(
			stats.toString() ).append( ": " ).append( getScore() ).toString();
}

/**
 * @see java.lang.Object#hashCode()
 */
public int hashCode() {
	return player.getPlayerId().hashCode();
}

/**
 * @see java.lang.Object#equals(java.lang.Object)
 */
public boolean equals( Object o ) {
	PlayerResults other = (PlayerResults) o;
	return player.equals( other.getPlayer() );
}

}
