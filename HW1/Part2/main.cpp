#include <SFML/Graphics.hpp>
#include <iostream>

int main() {
    sf::RenderWindow window(sf::VideoMode(640, 480), "Part 2: Moving a Sprite");

    sf::Texture texture;
    if (!texture.loadFromFile("../assets/boid-sm.png")) {
        std::cerr << "Failed to load sprite texture!" << std::endl;
        return -1;
    }

    sf::Sprite sprite(texture);


    // initial position
    sprite.setPosition(0, 0);

    const float speed = 5.0f;
    sf::Clock clock;

    while (window.isOpen()) {
        //close window
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        //move 
        if (clock.getElapsedTime().asMilliseconds() > 100) {
            sprite.move(speed, 0);
            clock.restart();
        }

        // teleport
        if (sprite.getPosition().x > 640) {
            sprite.setPosition(0, 0);
        }

        //set background color
        window.clear(sf::Color::White);
        window.draw(sprite);
        window.display();
    }

    return 0;
}

//compile process\
move to part2 folder\
type "make" in terminal to make executable file\
type ./main execut program\
use "make clean" clean object file and executable file


//scp -r "/Users/liuzitang/Desktop/CSC584/HW1" leslie@172.16.40.131:/home/leslie
