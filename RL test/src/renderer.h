#ifndef RENDERER_H
#define RENDERER_H

#include <SFML/Graphics.hpp>
#include "environment.h"
#include "agent.h"

class Renderer {
public:
    Renderer(int gridSize, int cellSize);
    void render(const Environment& env, const Agent& agent);
    bool pollWindowEvents();
    sf::RenderWindow window;


private:
    int gridSize;      // Number of rows/columns in the grid
    int cellSize;      // Pixel size of each cell
    sf::RectangleShape cellShape;
    sf::CircleShape agentShape;
};

#endif
