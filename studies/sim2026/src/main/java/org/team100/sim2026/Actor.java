package org.team100.sim2026;

public interface Actor {
    /**
     * Examine the game state and decide what to do. The returned runnable is
     * executed after all the actors have a chance to do the same.
     * 
     * TODO: replace "Runnable" with "Action"
     */
    Runnable step();

}
