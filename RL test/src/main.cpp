#include <iostream>
#include "environment.h"
#include "agent.h"
#include "q_learning.h"
#include "renderer.h"

int main() {
    srand(time(0)); // Random seed
    const int gridSize = 10;
    const int cellSize = 50; // Each cell is 50x50 pixels

    Environment env(gridSize, gridSize);
    Renderer renderer(gridSize, cellSize);

    QLearning ql(0.1, 0.9, 0.2); // (alpha, gamma, epsilon)
    ql.loadQTable("data/q_table.csv");

    for (int episode = 0; episode < 50; episode++) {
        env.reset();
        Agent agent(0, 0);
        auto state = agent.getPosition();
        auto goal = env.getGoalPosition();

        std::cout << "Episode " << episode << " Goal Position: (" 
                  << goal.first << ", " << goal.second << ")\n";

        for (int step = 0; step < 100; step++) {
            if (!renderer.pollWindowEvents()) {
                ql.saveQTable("data/q_table.csv"); // Save Q-table if window is closed
                return 0;
            }

            renderer.render(env, agent); // Render the grid and agent

            // Pass the goal position to the chooseAction function
            std::string action = ql.chooseAction(state, env, goal);
            std::cout << "Episode " << episode << ", Step " << step 
                      << ", Action: " << action << "\n";

            agent.move(action, env);
            auto newState = agent.getPosition();
            double reward = env.getReward(newState.first, newState.second);

            std::cout << "Reward: " << reward << " at (" << newState.first 
                      << ", " << newState.second << ")\n";

            ql.updateQTable(state, action, reward, newState, goal);

            // If goal is reached, log and break the episode
            if (env.isGoal(newState.first, newState.second)) {
                std::cout << "Goal reached at (" << newState.first << ", " 
                          << newState.second << ") in episode " << episode << "!\n";
                break;
            }

            state = newState;

            // Add a small delay for better visualization
            sf::sleep(sf::milliseconds(100));
        }

        // Save the Q-table at the end of each episode
        ql.saveQTable("data/q_table.csv");
        std::cout << "Q-table updated after Episode " << episode << "\n";
    }

    std::cout << "Training completed and Q-table saved." << std::endl;

    return 0;
}
