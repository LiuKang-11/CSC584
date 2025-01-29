#include "renderer.h"

// Constructor: Initialize the SFML window and shapes
Renderer::Renderer(int gridSize, int cellSize)
    : gridSize(gridSize), cellSize(cellSize),
      window(sf::VideoMode(gridSize * cellSize, gridSize * cellSize), "RL Visualization"),
      cellShape(sf::Vector2f(cellSize - 2, cellSize - 2)), // Add a border for better visuals
      agentShape(cellSize / 2.5) { // Agent is a circle smaller than a cell
    agentShape.setFillColor(sf::Color::Green);
}

// Render the grid, obstacles, goal, and agent
void Renderer::render(const Environment& env, const Agent& agent) {
    window.clear(sf::Color::White); // Clear the window with white color

    for (int row = 0; row < gridSize; row++) {
        for (int col = 0; col < gridSize; col++) {
            cellShape.setPosition(col * cellSize, row * cellSize);

            // Color cells based on their state
            if (env.isGoal(row, col)) {
                cellShape.setFillColor(sf::Color::Red); // Goal cell
            } else if (env.isObstacle(row, col)) {
                cellShape.setFillColor(sf::Color::Black); // Obstacle cell
            } else {
                cellShape.setFillColor(sf::Color::White); // Empty cell
            }

            // Draw each cell
            window.draw(cellShape);
        }
    }

    // Draw the agent
    auto agentPos = agent.getPosition();
    agentShape.setPosition(agentPos.second * cellSize + cellSize / 4,
                           agentPos.first * cellSize + cellSize / 4);
    window.draw(agentShape);

    window.display(); // Display the updated window
}

// Poll window events and return whether the window should remain open
bool Renderer::pollWindowEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
            return false; // Window is closed
        }
    }
    return true; // Window remains open
}
