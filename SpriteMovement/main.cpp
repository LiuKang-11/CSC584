#include<SFML/Graphics.hpp>
#include<iostream>
#define SPEED 0.12

int main()
{
    // Creating the window and intial variables
    sf::RenderWindow window(sf::VideoMode(640, 480), "HW1 Part 1");
    sf::Sprite sprite;
    sf::Texture texture;

    if (!texture.loadFromFile("assets/boid.png", sf::IntRect(0, 0, 930, 627)))
    {
        std::cout<<"Error, unable to load file";
        exit (0);
    }
    sprite.setTexture(texture);
    sprite.setScale(0.05,0.05);
    sprite.setPosition(0.f,0.f);

    sf::Vector2f position = sprite.getPosition();
    sf::Event event;

    while (window.isOpen())
    {
        while (window.pollEvent(event))
        {
            
            if (event.type == sf::Event::Closed){
                std::cout<<"Closing Window";
                window.close();
            }
                
        }

        // Clears the window with green color
        window.clear(sf::Color::White);

        // Position is moved until we hit the edge of the screen
        if (position.x < 640){
            sprite.move(sf::Vector2f(SPEED, 0));  
            position.x += SPEED;
        }
        else{
            sprite.setPosition(0.f,0.f);
            position = sprite.getPosition();
        }

        // Drawing the sprite
        window.draw(sprite);

        // Displaying the window
        window.display();
    }

    return 0;
}