#include <SFML/Graphics.hpp>

int main() {
    // Create a RenderWindow
    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML 2.x Window");

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear(sf::Color::Black);
        window.display();
    }

    return 0;
}